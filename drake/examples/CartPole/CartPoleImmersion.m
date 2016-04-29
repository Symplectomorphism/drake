classdef CartPoleImmersion < DrakeSystem
  % technically not the same controller as Spong96.  He goes through PFL.
  % I'm just doing energy shaping on the non-PFL'd system here.
  
  properties
    p  % plant
    k1 = 3; k2 = 4; gamma = 1;
  end
  
  methods
    function obj = CartPoleImmersion(plant)
      obj = obj@DrakeSystem(0,0,4,1,true,true);
      if (nargin>0)
        typecheck(plant,'CartPolePlant');
        obj.p = plant;
        obj = obj.setInputFrame(plant.getStateFrame);
        obj = obj.setOutputFrame(plant.getInputFrame);
      end
    end
    
    function u = output(obj,t,~,x)
      u = 1/(obj.k2-1)*(obj.gamma*(x(2)+obj.k1*x(3)+1/cos(x(3))*...
          obj.k2*x(4))+obj.k1*x(4)+obj.k2*tan(x(3))*(x(4)^2/cos(x(3))+1));
%       scope('CartPole','phase',x(2),x(4),struct('resetOnXval',false));

%       function u = sat(u,lim)
%         u = max(min(u,lim),-lim);
%       end
    end
  end
  
  methods (Static)
    function run()
      cp = CartPolePlant();
      c = CartPoleImmersion(cp);
      v = CartPoleVisualizer(cp);
      sys = feedback(cp,c);

%       for i=1:5
      x = simulate(sys,[0 5]);
      playback(v,x);
%       end
    end
  end
end
