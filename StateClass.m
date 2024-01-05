classdef StateClass < handle
    properties
        state = 0;
    end
    methods
        function set.state(obj,val)
            obj.state = val;
        end
    end
end