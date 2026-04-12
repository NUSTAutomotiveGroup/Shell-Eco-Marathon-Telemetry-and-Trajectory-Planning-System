classdef physics_prototype_beginCheck < matlab.System
% Python def beginCheck imported from python module physics_prototype.py
methods(Access = protected)
function validateInputsImpl(obj, varargin)
if ~isempty(varargin{1})
    validateattributes(varargin{1}, {'double'}, {'size',[1 1]});
end
end
function setupImpl(obj)
if coder.target('MATLAB')
    py.importlib.import_module('physics_prototype');
end
end
function stepImpl(obj,on_idle)
    py.physics_prototype.beginCheck(on_idle);
end
end
methods(Static, Access = protected)
end
end
