classdef physics_prototype_distancePerTime < matlab.System
% Python def distancePerTime imported from python module physics_prototype.py
methods(Access = protected)
function setupImpl(obj)
if coder.target('MATLAB')
    py.importlib.import_module('physics_prototype');
end
end
function [distance] = stepImpl(obj)
    distance = double(py.physics_prototype.distancePerTime());
end
function varargout = getOutputDataTypeImpl(obj)
    varargout{1} = 'double';
end
function varargout = getOutputSizeImpl(obj)
    varargout{1} = [1 1];
end
function varargout = isOutputComplexImpl(obj)
    varargout{1} = false;
end
function varargout = isOutputFixedSizeImpl(obj)
    varargout{1} = true;
end
end
methods(Static, Access = protected)
end
end
