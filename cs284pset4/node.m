classdef node
    
    properties
        parent;
        children;
        location;
        actions;
        cost;
    end
    
    methods
        function obj = node(parent,children,location,actions,cost)
            if(nargin > 0)
                obj.parent = parent;
                obj.children   = children;
                obj.location    = location;
                obj.actions  = actions;
                obj.cost   = cost;
            end
        end
        
    end
    
end

