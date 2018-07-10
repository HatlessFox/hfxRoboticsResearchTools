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

    function coord_shift = find_coord_shift(this, ref_scan, cur_scan, coord_id)
      min_v = min([ref_scan.cart(:, coord_id); cur_scan.cart(:, coord_id)]);
      max_v = max([ref_scan.cart(:, coord_id); cur_scan.cart(:, coord_id)]);
      hist_cbin_range = min_v:this.chist_res:max_v;

      [ref_hist _] = hist(ref_scan.cart(:, coord_id), hist_cbin_range);
      ref_hist = normalize(ref_hist, "peak");
      [cur_hist _] = hist(cur_scan.cart(:, coord_id), hist_cbin_range);
      cur_hist = normalize(cur_hist, "peak");

      max_abs_lag = floor(size(ref_hist, 1) / 2);
      [corr lags] = xcorr(cur_hist, ref_hist, max_abs_lag);

      # TODO: find several hypotheses?
      [_ max_lag_i] = max(corr);
      coord_shift = lags(max_lag_i) * this.chist_res;

      if this.is_debug_mode # show hists
        title = sprintf("[DEBUG] %d Coordinate Histograms", coord_id);
        OrigAhsm.show_histograms(title, ref_hist, cur_hist,
                                 lags, corr, this.chist_res, coord_shift);
      endif
    endfunction

    function main_dir = find_main_direction(this, ref_scan, ref_pose)
      ref_tscan = transform_scan(ref_scan, ref_pose);
      ares = this.ahist_res;
      [ref_ahist cbins] = this.make_angle_histogram(ref_tscan);

       # (?) find several
      [_ main_dir_i] = max(ref_ahist);
      main_dir = cbins(main_dir_i);
    endfunction

    function coords = find_translation(this, ref_scan, ref_pose,
                                       cur_scan, cur_pose)
      main_dir = this.find_main_direction(ref_scan, ref_pose);
      main_dir_dpose = [0 0 main_dir];

      rtref_scan = transform_scan(ref_scan, ref_pose + main_dir_dpose);
      rtcur_scan = transform_scan(cur_scan, cur_pose + main_dir_dpose);

      if this.is_debug_mode
        figure("name", "[DEBUG] Main Direction Alignment", "numbertitle", "off",
             "menubar", "none", "toolbar", "none");
        hold on;
        axis equal;
        grid on;
        display_scan2D(rtref_scan, 'b');
        display_scan2D(rtcur_scan, 'r');
      endif

      d_x = this.find_coord_shift(rtcur_scan, rtref_scan, 1);
      d_y = this.find_coord_shift(rtcur_scan, rtref_scan, 2);

      rot = createRotation(-main_dir)(1:2, 1:2);
      coords = rot * [d_x; d_y]
    endfunction

    function ang = find_rotation(this, ref_scan, ref_pose, cur_scan)
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

      d_pose(3) = this.find_rotation(ref_scan, ref_pose, cur_scan);
      d_pose(1:2) = this.find_translation(ref_scan, ref_pose,
                                          cur_scan, ref_pose + d_pose);
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
      # TODO: print max on the figure
    endfunction
  endmethods

endclassdef
