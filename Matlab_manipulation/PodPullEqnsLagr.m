3%
% Matlab code for the Course:
%
%     Modelling and Simulation of Mechatronics System
%
% by
% Enrico Bertolazzi
% Dipartimento di Ingegneria Industriale
% Universita` degli Studi di Trento
% email: enrico.bertolazzi@unitn.it
%
classdef PodPullEqnsLagr < DAC_ODEclass
  properties (SetAccess = protected, Hidden = true)
    xA;
    yA;
    xD;
    yD;
    L1;
    L2;
    L3;
    H3;
    L__wing;
    d__tip;
    m__pist;
    m__link;
    m__wing;
    F__drag;
    F__down;
    s__desired;
    k;
    g;
    eta;
    omega;
    npos;
  end
  methods
    function self = PodPullEqnsLagr( xA,yA,xD,yD,L1,L2,L3,H3,L__wing,d__tip,m__pist,m__link,m__wing,F__drag,F__down,s__desired,g,k )
      neq  = 8;
      ninv = 2;
     
      self@DAC_ODEclass( 'PodPullEqnsLagr', neq, ninv );
      self.xA = xA;
      self.yA = yA;
      self.xD = xD;
      self.yD = yD;
      self.L1 = L1;
      self.L2 = L2;
      self.L3 = L3;
      self.H3 = H3;
      self.L__wing = L__wing;
      self.d__tip = d__tip;
      self.m__pist = m__pist;
      self.m__link = m__link;
      self.m__wing = m__wing;
      self.F__drag = F__drag;
      self.F__down = F__down;
      self.s__desired = s__desired;
      self.g = g;
      self.k = k;
      self.eta = 1;
      self.omega = 10;
      self.npos = 3;
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res__f = f( self, t, vars__ )
      % extract parameters
      xA = self.xA;
      yA = self.yA;
      xD = self.xD;
      yD = self.yD;
      L1 = self.L1;
      L2 = self.L2;
      L3 = self.L3;
      H3 = self.H3;
      L__wing = self.L__wing;
      d__tip = self.d__tip;
      m__pist = self.m__pist;
      m__link = self.m__link;
      m__wing = self.m__wing;
      F__drag = self.F__drag;
      F__down = self.F__down;
      s__desired = self.s__desired;
      g = self.g;
      k = self.k;
      eta = self.eta;
      omega = self.omega;




    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res__DfDx = DfDx( self, t, vars__ )
    % extract parameters
      xA = self.xA;
      yA = self.yA;
      xD = self.xD;
      yD = self.yD;
      L1 = self.L1;
      L2 = self.L2;
      L3 = self.L3;
      H3 = self.H3;
      L__wing = self.L__wing;
      d__tip = self.d__tip;
      m__pist = self.m__pist;
      m__link = self.m__link;
      m__wing = self.m__wing;
      F__drag = self.F__drag;
      F__down = self.F__down;
      s__desired = self.s__desired;
      g = self.g;
      k = self.k;
      eta = self.eta;
      omega = self.omega;
    
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res__DfDt = DfDt( self, t, vars__ )

      % extract states
      t = vars__(1);

      % evaluate function
      
      % store on output
      res__DfDt = zeros(8,1);
    end

    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res__h = h( self, t, vars__ )
      % extract parameters
      xA = self.xA;
      yA = self.yA;
      xD = self.xD;
      yD = self.yD;
      L1 = self.L1;
      L2 = self.L2;
      L3 = self.L3;
      H3 = self.H3;
      L__wing = self.L__wing;
      d__tip = self.d__tip;
      m__pist = self.m__pist;
      m__link = self.m__link;
      m__wing = self.m__wing;
      F__drag = self.F__drag;
      F__down = self.F__down;
      s__desired = self.s__desired;
      g = self.g;
      k = self.k;
      eta = self.eta;
      omega = self.omega;

      
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res__DhDx = DhDx( self, t, vars__ )
      % extract parameters
      xA = self.xA;
      yA = self.yA;
      xD = self.xD;
      yD = self.yD;
      L1 = self.L1;
      L2 = self.L2;
      L3 = self.L3;
      H3 = self.H3;
      L__wing = self.L__wing;
      d__tip = self.d__tip;
      m__pist = self.m__pist;
      m__link = self.m__link;
      m__wing = self.m__wing;
      F__drag = self.F__drag;
      F__down = self.F__down;
      s__desired = self.s__desired;
      g = self.g;
      k = self.k;
      eta = self.eta;
      omega = self.omega;

     
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res__DhDt = DhDt( self, t, vars__ )

      % extract states
      t = vars__(1);

      % evaluate function
      
      % store on output
      res__DhDt = zeros(2,1);
    end

    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function plot( self, t, Z )
      % extract parameters
      xA = self.xA;
      yA = self.yA;
      xD = self.xD;
      yD = self.yD;
      L1 = self.L1;
      L2 = self.L2;
      L3 = self.L3;
      H3 = self.H3;
      L__wing = self.L__wing;
      d__tip = self.d__tip;
      
      display("Soluz");
      display(Z(1));
      display(Z(2));
      display(Z(3));

      hold off;
      % L1
      drawLine( xA-Z(1), yA, xA-Z(1)+L1, yA, 'LineWidth', 5, 'Color', 'black' );
      hold on;
      axis equal;
      % L2
      drawLine(xA-Z(1)+L1, yA, xA-Z(1)+L1+L2*cos(Z(2)),yA+L2*sin(Z(2)),'LineWidth', 5, 'Color', 'black' );
      % H3
      drawLine(xA-Z(1)+L1+L2*cos(Z(2)),yA+L2*sin(Z(2)), xD-(L__wing-d__tip-L3)*cos(Z(3)),yD-(L__wing-d__tip-L3)*sin(Z(3)),'LineWidth', 5, 'Color', 'black' );
      % flap wing
      drawLine(xD+d__tip*cos(Z(3)), yD+d__tip*sin(Z(3)), xD-(L__wing-d__tip)*cos(Z(3)),yD-(L__wing-d__tip)*sin(Z(3)),'LineWidth', 5, 'Color', 'r' );
    
      title(sprintf('time=%5.2g',t/100));
    end
  end
end