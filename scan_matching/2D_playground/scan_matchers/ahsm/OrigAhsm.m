classdef OrigAhsm
  # Angle Histogram Scan Matcher
  #   This method uses a 1D signal-type representation of the scan points.
  #   (1)
  #     The vector delimited by every two consecutive points of the scan
  #     is computed and represented in polar  coordinates with respect to
  #     the global frame of reference.
  #   (2)
  #     Then, a histogram of their angles is computed as a 1D signal
  #     which is invariant to translation but not to rotation.
  #   (3)
  #     Given two rotated scans, the rotation angle can be computed as
  #     the signal shift which maximizes the cross correlation of
  #     the corresponding scan-signals.
  #   (4) * 2
  #     Once the  rotation has been solved, the same procedure may be
  #     applied for the x and y to solve for the translation.
  #
  # Based on:
  #   "Keeping Track of Position and Orientation of Moving Indoor Systems
  #    by Correlation of Range-Finder Scans"
  #   by Gerhard Wei$, Christopher Wetzler, Ewald von Puttkamer

  properties # TODO: (SetAccess = immutable)
    is_debug_mode = false;
    ahist_res = deg2rad(1);
    chist_res = 0.25; # cm
  endproperties
  methods
    function this = OrigAhsm(varargin)
      p = inputParser();
      p.addParameter("debug", false, @isbool);
      p.addParameter("ahist_res", deg2rad(1), @isfloat);
      p.addParameter("chist_res", 0.25, @isfloat);
      p.parse(varargin{:});

      this.is_debug_mode = p.Results.debug;
      this.ahist_res = p.Results.ahist_res; # angle histogram resolution
      this.chist_res = p.Results.chist_res; # coord histogram resolution
    endfunction

    function [h b] = make_angle_histogram(this, scan)
      a_step = this.ahist_res;
      pts_nm = size(scan.cart, 1);
      angles = zeros(pts_nm - 1, 1);
      for i = 1:pts_nm - 1
        d_cart = scan.cart(i, :) - scan.cart(i + 1, :);
        # TODO: filter holes
        ## if (1 < d_cart * d_cart')
        ##   continue;
        ## end
        ang = atan2(d_cart(1), d_cart(2));
        if (pi - a_step/2 < ang) # ... | .... | .. -> .. 360=0
          ang = -pi;
        endif
        angles(i) = ang;
      endfor

      [h b] = hist(angles, -pi:a_step:pi - a_step);
      h = normalize(h, "peak");
    endfunction


    function ang = find_rotation(this, ref_scan, ref_pose,
                                       cur_scan, cur_pose)
      ref_tscan = transform_scan(ref_scan, ref_pose);
      cur_tscan = transform_scan(cur_scan, ref_pose);

      # Generate the Angle histogram of both scans and normalize
      ares = this.ahist_res;
      [ref_ahist ref_cbins] = this.make_angle_histogram(ref_tscan);
      [cur_ahist cur_cbins] = this.make_angle_histogram(cur_tscan);

      [a_xcorr a_lags] = cxcorr(cur_ahist, ref_ahist);
      a_xcorr = normalize(a_xcorr, "peak");
      # (?) to cut off the borders with .*hann(len(ang_corr)) 
      # (?) smooth withfastsmooth(ang_corr, 10); (breakes a peaky function)

      # (?) handle several maxes (noisy scan case)
      [mx mx_ang_corr_lag_i] = max(a_xcorr);
      ang = cur_cbins(mx_ang_corr_lag_i);

      if this.is_debug_mode
        OrigAhsm.show_histograms("[DEBUG] Angle Histograms",
                                 ref_ahist, cur_ahist,
                                 a_lags, a_xcorr, rad2deg(ares), ang);
      endif
    endfunction

    function [d_pose, prob] = find_transformation(this,
                                                  ref_scan, ref_pose,
                                                  cur_scan, cur_pose)
      d_pose = [0 0 0];
      prob = 1;

      d_pose(3) = this.find_rotation(ref_scan, ref_pose, cur_scan, cur_pose);
      ##########################################################################
      # Estimate translation

      #TODO: review
      ref_tscan = transform_scan(ref_scan, ref_pose);
      ares = this.ahist_res;
      ref_ahist = this.make_angle_histogram(ref_tscan);
      max_abs_lag = size(ref_ahist, 1) / 2;

      # Find main direction
      [_ main_dir_lag_i] = max(ref_ahist); # (?) find several
      main_dir = (main_dir_lag_i - max_abs_lag - 1) * this.ahist_res;
      main_dir_pose = [0 0 main_dir];
  
      rtref_scan = transform_scan(ref_scan, ref_pose + main_dir_pose);
      rtcur_scan = transform_scan(cur_scan, ref_pose + d_pose + main_dir_pose);

      if this.is_debug_mode # TODO
        ## figure;
        ## hold on
        ## axis equal
        ## grid on
        ## display_scan(rtref_scan.cart,'b',0);
        ## display_scan(rtcur_scan.cart,'g',0);
      end

      #TODO: hist(angles, -pi:resolution:pi - resolution);
      min_x = min(min(rtref_scan.cart(:,1)),min(rtcur_scan.cart(:,1)));
      max_x = max(max(rtref_scan.cart(:,1)),max(rtcur_scan.cart(:,1)));
      ref_xhist = make_histogram(rtref_scan.cart(:, 1), min_x, max_x,
                                 this.chist_res);
      cur_xhist = make_histogram(rtcur_scan.cart(:, 1), min_x, max_x,
                                 this.chist_res);

      min_y = min(min(rtref_scan.cart(:,2)),min(rtcur_scan.cart(:,2)));
      max_y = max(max(rtref_scan.cart(:,2)),max(rtcur_scan.cart(:,2)));
      ref_yhist = make_histogram(rtref_scan.cart(:, 2), min_y, max_y,
                                 this.chist_res);
      cur_yhist = make_histogram(rtcur_scan.cart(:, 2), min_y, max_y,
                                 this.chist_res);

      max_abs_xlag = floor(size(ref_xhist, 1) / 2);
      [x_xcorr x_lags] = xcorr(cur_xhist, ref_xhist, max_abs_xlag);

      max_abs_ylag = floor(size(ref_yhist, 1) / 2);
      [y_xcorr y_lags] = xcorr(cur_yhist, ref_yhist, max_abs_ylag);

      # TODO: find several hypotheses?
      [_ x_max_lag_i] = max(x_xcorr);
      [_ y_max_lag_i] = max(y_xcorr);
      d_x_dir = x_lags(x_max_lag_i) * this.chist_res;
      d_y_dir = y_lags(y_max_lag_i) * this.chist_res;

      if this.is_debug_mode # show hists
        OrigAhsm.show_histograms("[DEBUG] X Coordinate Histograms",
                                 ref_xhist, cur_xhist,
                                 x_lags, x_xcorr, this.chist_res, d_x_dir);
        OrigAhsm.show_histograms("[DEBUG] Y Coordinate Histograms",
                                 ref_yhist, cur_yhist,
                                 y_lags, y_xcorr, this.chist_res, d_y_dir);
      end

      # TODO: rotation
      main_dir = 0;
      rt = [cos(main_dir) -sin(main_dir); sin(main_dir) cos(main_dir)];
      d_pose(1:2) = rt * [d_x_dir; d_y_dir];

    endfunction
  endmethods

  methods(Static)
    function [] = show_histograms(figure_title, ref_hist, cur_hist,
                                  corr_lags, corr, resolution, best)
      # show hists
      # TODO: figure positioning is hardcoded
      figure("name", figure_title, "numbertitle", "off",
             "menubar", "none", "toolbar", "none", "position", [0 0 700 700]);
      subplot(3, 1, 1);
      plot(1:size(ref_hist, 1), ref_hist);
      title("Reference");
      subplot(3, 1, 2);
      plot(1:size(cur_hist, 1), cur_hist);
      title("Current");
      # show xcorr
      subplot(3, 1, 3);
      plot(corr_lags * resolution, corr);
      title("XCorr");
      # TODO: print max
    endfunction
  endmethods

endclassdef
