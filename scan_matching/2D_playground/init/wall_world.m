classdef wall_world
  properties
    walls = [];
  end
  methods

    function obj = wall_world(varargin)
      # TODO: interactive setup
      obj.walls = wall_world.gen_rect(0, 0, 20, 20, 0)';
    end

    function disp(obj)
      figure;
      plot(obj.walls(1,:),obj.walls(2,:),'LineWidth',3);
      axis equal;
    end
  end # non-static methods
    
  methods (Static)

    #######################
    # Structured generation

    function [pol] = gen_rect(x0,y0,a,b,theta) % angular positions of vertices
      rect = [x0-a/2, y0+b/2;  # top-left
              x0+a/2, y0+b/2;  # top-right
              x0+a/2, y0-b/2;  # bot-right
              x0-a/2, y0-b/2]; # bot-left
      # TODO: fix rotation (wrt x0 y0)
      T = [cos(theta) -sin(theta);
           sin(theta)  cos(theta)];
      for i = 1:size(rect,1)
        rotated_rect(i, :) = T*rect(i,:)';
      end
      pol = wall_world.convpoly2cw(rotated_rect);
      pol = vertcat(pol, pol(1, :));
    end

    function [poly] = convpoly2cw(poly)
      # TODO: generic version
      xs = poly(:, 1)';
      ys = poly(:, 2)';
      angles = atan2(ys - mean(ys), xs - mean(ys));
      [~, order] = sort(angles);
      poly = [xs(order); ys(order)]';
    end

    function [pol] = draw_ellipse(x0,y0,a,b,theta)
      # TODO: fix angle rotation
      % angular positions of vertices
      t = linspace(0, 2*pi, 30);
      for i = 1:length(x0)
        % pre-compute rotation angles (given in degrees)
        cot = cosd(theta);
        sit = sind(theta);
        % compute position of points used to draw current ellipse
        xt = x0(i) + a(i) * cos(t) * cot - b(i) * sin(t) * sit;
        yt = y0(i) + a(i) * cos(t) * sit + b(i) * sin(t) * cot;
      end
      pol = wall_world.convpoly2cw([xt;yt]');
      pol = vertcat(pol, pol(1, :));
    end

    %Generator
    function polygon = random_world(ell_sigma)
      npoly = round(rand*10)+4;
      polygon = [];
      for n = 1:npoly
        type = rand;
        op = merge(0.8 < rand, 'ab', 'or');

        this_pol = [];
        x0_r = rand*5+1;
        y0_r = rand*5+1;
        a_r = rand*3+1;
        b_r = rand*3+1;
        theta_r = 0; #rand*(pi/2);

        if type > ell_sigma %rect
          this_pol = wall_world.gen_rect(x0_r,y0_r,a_r,b_r,theta_r);
        else %ellipse
          this_pol = wall_world.draw_ellipse(x0_r,y0_r,a_r/2,b_r/2,theta_r);
        end

        if ~isempty(polygon)
          [poly_x, poly_y] = oc_polybool(polygon, this_pol, 'or');
          polygon = [poly_x poly_y];
        else
          polygon = this_pol;
        end
      end
      polygon = polygon';
    end

  end # static methods
end # classdef
