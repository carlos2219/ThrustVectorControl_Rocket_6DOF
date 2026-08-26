%% ========================================================================
%  Simulink -> JSON Netlist Extractor (v3)
%  v2: detecta y reporta explicitamente los puertos SIN CONECTAR.
%  v3: agrega el InitFcn/callbacks del modelo y el codigo fuente completo
%      de todos los bloques MATLAB Function (Stateflow EMChart), para que
%      un LLM tenga tanto la topologia como la logica real del modelo.
%
%  Cambio de fondo (v2): en vez de basarse en PortConnectivity (que agrupa
%  src/dst de forma ambigua y silenciosamente descarta puertos vacios),
%  este script enumera los puertos via PortHandles y consulta la
%  propiedad 'Line' de cada uno. Line == -1  =>  puerto desconectado.
%
%  Nota tecnica (v3): un bloque "MATLAB Function" no es una funcion normal
%  de Simulink -- internamente es un chart de Stateflow (clase
%  Stateflow.EMChart). Su codigo NO se puede leer con get_param(block,...);
%  hay que consultar el arbol de Stateflow via sfroot() y emparejar el
%  chart con el bloque por su ruta completa (Path).
%  ========================================================================
clc; clear;

% 1. Configura el nombre de tu modelo
modelName = 'SIM_model';
open_system(modelName);

% 2. Obtener todos los bloques del modelo de forma profunda
blocks = find_system(modelName, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
netlist = struct();

% Tipos de puerto a inspeccionar en cada bloque
portFieldNames  = {'Inport','Outport','Enable','Trigger','State','Ifaction','LConn','RConn'};
% Puertos tipo "salida": pueden tener multiples destinos (ramas de senal)
outputLikeFields = {'Outport','RConn'};

totalUnconnected = 0;

% --- Callbacks / InitFcn a nivel de MODELO -----------------------------
modelCallbackFields = {'InitFcn','StartFcn','StopFcn','PreLoadFcn','PostLoadFcn','CloseFcn'};
modelInfo = struct();
modelInfo.ModelName = modelName;
for c = 1:length(modelCallbackFields)
    cbName = modelCallbackFields{c};
    try
        modelInfo.(cbName) = get_param(modelName, cbName);
    catch
        modelInfo.(cbName) = '';
    end
end

% --- Mapa de codigo fuente de todos los bloques MATLAB Function --------
% Se construye una sola vez recorriendo todos los charts EMChart (MATLAB
% Function) del modelo y se indexa por la ruta completa del bloque, para
% poder adjuntar el codigo a cada bloque dentro del loop principal.
matlabFnCode = containers.Map('KeyType', 'char', 'ValueType', 'char');
try
    rt = sfroot;
    emCharts = rt.find('-isa', 'Stateflow.EMChart');
    for c = 1:length(emCharts)
        try
            chartObj  = emCharts(c);
            chartPath = chartObj.Path;          % ruta completa del bloque contenedor
            chartCode = chartObj.Script;        % codigo MATLAB del bloque
            matlabFnCode(chartPath) = chartCode;
        catch
            continue;
        end
    end
catch
    % sfroot puede fallar si no hay licencia/uso de Stateflow en el modelo
end

for i = 1:length(blocks)
    bHandle = blocks{i};
    try
        bName   = get_param(bHandle, 'Name');
        bType   = get_param(bHandle, 'BlockType');
        bParent = get_param(bHandle, 'Parent');
    catch
        continue; % Saltarse elementos virtuales o inaccesibles
    end

    % Limpiar el nombre para usarlo como llave valida en la estructura JSON
    cleanName = regexprep(bName, '[^a-zA-Z0-9_]', '_');
    if isempty(cleanName) || ~isletter(cleanName(1)) || iskeyword(cleanName)
        cleanName = ['block_' num2str(i)];
    end

    inputs      = {};
    outputs     = {};
    unconnected = {};

    try
        ph = get_param(bHandle, 'PortHandles');
    catch
        ph = struct();
    end

    for f = 1:length(portFieldNames)
        fName = portFieldNames{f};
        if ~isfield(ph, fName) || isempty(ph.(fName))
            continue;
        end
        handles = ph.(fName);

        for k = 1:length(handles)
            portH = handles(k);

            % Numero de puerto (algunos puertos especiales no lo exponen)
            try
                portNum = get_param(portH, 'PortNumber');
            catch
                portNum = k;
            end

            try
                lineH = get_param(portH, 'Line');
            catch
                lineH = -1;
            end

            if lineH == -1
                % --- PUERTO SIN CONECTAR: esto es lo que faltaba ---
                unconnected{end+1} = struct('PortType', fName, 'PortNumber', portNum); %#ok<AGROW>
                totalUnconnected = totalUnconnected + 1;
                continue;
            end

            isOutputLike = any(strcmp(fName, outputLikeFields));

            try
                if isOutputLike
                    dests = getLineDestinations(lineH);
                    for d = 1:length(dests)
                        outputs{end+1} = struct( ...
                            'PortType', fName, 'PortNumber', portNum, ...
                            'ToBlock', dests(d).BlockName, 'ToPort', dests(d).PortNumber); %#ok<AGROW>
                    end
                else
                    srcPortH  = get_param(lineH, 'SrcPortHandle');
                    srcParent = get_param(srcPortH, 'Parent'); % ruta completa del bloque origen
                    [~, srcName] = fileparts(char(srcParent));
                    srcPortNum = get_param(srcPortH, 'PortNumber');
                    inputs{end+1} = struct( ...
                        'PortType', fName, 'PortNumber', portNum, ...
                        'FromBlock', srcName, 'FromPort', srcPortNum); %#ok<AGROW>
                end
            catch
                % Si la conexion no se puede resolver (p.ej. puertos fisicos
                % de Simscape con un objeto de linea distinto), no se pierde
                % el dato: se marca como conectado-pero-no-resuelto en vez
                % de desaparecer silenciosamente como en la version original.
                unconnected{end+1} = struct('PortType', fName, 'PortNumber', portNum, ...
                    'Note', 'connected_but_unresolved'); %#ok<AGROW>
            end
        end
    end

    fullBlockPath = [char(bParent) '/' char(bName)];

    % Callback InitFcn propio del bloque (mask callback), si existe y no
    % esta vacio -- distinto del InitFcn del modelo.
    try
        blockInitFcn = get_param(bHandle, 'InitFcn');
    catch
        blockInitFcn = '';
    end

    uniqueKey = ['id_' num2str(i) '_' cleanName];
    netlist.(uniqueKey).Type             = bType;
    netlist.(uniqueKey).FullName         = fullBlockPath;
    netlist.(uniqueKey).Inputs           = inputs;
    netlist.(uniqueKey).Outputs          = outputs;
    netlist.(uniqueKey).UnconnectedPorts = unconnected;

    if ~isempty(blockInitFcn)
        netlist.(uniqueKey).BlockInitFcn = blockInitFcn;
    end

    % Si este bloque es una MATLAB Function, adjuntar su codigo fuente
    if isKey(matlabFnCode, fullBlockPath)
        netlist.(uniqueKey).IsMatlabFunction = true;
        netlist.(uniqueKey).MatlabFunctionCode = matlabFnCode(fullBlockPath);
    end
end

% 3. Ensamblar resultado final: info de modelo + netlist de bloques
result = struct();
result.ModelInfo = modelInfo;
result.Blocks = netlist;

% 4. Convertir a JSON y guardar en un archivo
jsonStr = jsonencode(result, 'PrettyPrint', true);
fid = fopen('simulink_netlist.json', 'w');
fprintf(fid, '%s', jsonStr);
fclose(fid);

fprintf(['Netlist JSON exportado con exito: %d bloques procesados, %d puertos sin conectar, ' ...
    '%d funciones MATLAB extraidas.\n'], length(blocks), totalUnconnected, matlabFnCode.Count);


%% ------------------------------------------------------------------
function dests = getLineDestinations(lineH)
% Recorre una linea de Simulink (incluyendo sus ramas / branches) y
% devuelve todos los puertos de destino como struct array con
% campos BlockName y PortNumber. Necesario porque una salida puede
% alimentar a varios bloques a la vez (senal ramificada).
    dests = struct('BlockName', {}, 'PortNumber', {});

    try
        dstPortH = get_param(lineH, 'DstPortHandle');
        if dstPortH ~= -1
            dstParent = get_param(dstPortH, 'Parent');
            [~, dstName] = fileparts(char(dstParent));
            dstPortNum = get_param(dstPortH, 'PortNumber');
            dests(end+1) = struct('BlockName', dstName, 'PortNumber', dstPortNum);
        end
    catch
    end

    try
        children = get_param(lineH, 'LineChildren');
        for c = 1:length(children)
            dests = [dests, getLineDestinations(children(c))]; %#ok<AGROW>
        end
    catch
    end
end