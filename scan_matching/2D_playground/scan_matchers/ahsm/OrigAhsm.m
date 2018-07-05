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

    function [d_pose, prob] = find_transformation(this,
                                                  ref_scan, ref_pose,
                                                  cur_scan, cur_pose)
      d_pose = [0 0 0];
      prob = 1;

      ##########################################################################
      # Estimate rotation

      # transform to the reference frame
      tref_scan = transform_scan(ref_scan, ref_pose);
      tcur_scan = transform_scan(cur_scan, ref_pose);

      # Generate the Angle histogram of both scans and normalize
      ref_ahist = normalize(make_angle_histogram(tref_scan, this.ahist_res),
                            "peak");
      cur_ahist = normalize(make_angle_histogram(tcur_scan, this.ahist_res),
                            "peak");

      if this.is_debug_mode # show hists
        # TODO: figure positioning is hardcoded
        figure("name", "[DEBUG] Angle Histograms", "numbertitle", "off",
               "menubar", "none", "toolbar", "none", "position", [0 0 700 700]);
        subplot(3, 1, 1);
        plot(1:size(ref_ahist, 1), ref_ahist);
        title("Reference");
        subplot(3, 1, 2);
        plot(1:size(cur_ahist, 1), cur_ahist);
        title("Current");
      end

      max_abs_lag = size(ref_ahist, 1) / 2;
      [ang_corr lags] = xcorr(cur_ahist, ref_ahist, max_abs_lag);
      ang_corr = normalize(ang_corr, "peak");

      # (?) to cut off the borders with .*hann(len(ang_corr)) 
      # (?) smooth withfastsmooth(ang_corr, 10); (breakes a peaky function)
      if this.is_debug_mode # show xcorr
        subplot(3, 1, 3);
        plot(lags, ang_corr);
        title("XCorr")
      end

      # findLocalMaxima(ang_corr, 3) (? BUGGY)
      [mx mx_ang_corr_lag_i] = max(ang_corr);
      # to [-180, 180)
      d_pose(3) = (mx_ang_corr_lag_i - max_abs_lag - 1) * this.ahist_res;
      if this.is_debug_mode # print rotation
        rotation = rad2deg(d_pose(3));
      end

      ##########################################################################
      # Estimate translation

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
  
      bounds = [min(min(rtref_scan.cart(:,1)),min(rtcur_scan.cart(:,1))), ...
                max(max(rtref_scan.cart(:,1)),max(rtcur_scan.cart(:,1)));
                min(min(rtref_scan.cart(:,2)),min(rtcur_scan.cart(:,2))), ...
                max(max(rtref_scan.cart(:,2)),min(rtcur_scan.cart(:,2)))];

      [ref_xhist, ref_yhist] = make_coord_histograms(rtref_scan, bounds,
                                                     this.chist_res);
      [cur_xhist, cur_yhist] = make_coord_histograms(rtcur_scan, bounds,
                                                     this.chist_res);

      max_abs_xlag = floor(size(ref_xhist, 1) / 2);
      [x_xcorr x_lags] = xcorr(cur_xhist, ref_xhist, max_abs_xlag);

      max_abs_ylag = floor(size(ref_yhist, 1) / 2);
      [y_xcorr y_lags] = xcorr(cur_yhist, ref_yhist, max_abs_ylag);

      if this.is_debug_mode # show hists
        # Mult lag by Resolution
        # TODO: figure positioning is hardcoded
        figure("name", "[DEBUG] X Coordinate Histograms", "numbertitle", "off",
               "menubar", "none", "toolbar", "none", "position", [0 0 700 700]);
        subplot(3, 1, 1);
        plot(1:size(ref_xhist, 1), ref_xhist);
        title("Reference");
        subplot(3, 1, 2);
        plot(1:size(cur_xhist, 1), cur_xhist);
        title("Current");
        subplot(3, 1, 3);
        plot(x_lags, x_xcorr);
        title("XCorr")

        figure("name", "[DEBUG] Y Coordinate Histograms", "numbertitle", "off",
               "menubar", "none", "toolbar", "none", "position", [0 0 700 700]);
        subplot(3, 1, 1);
        plot(1:size(ref_yhist, 1), ref_yhist);
        title("Reference");
        subplot(3, 1, 2);
        plot(1:size(cur_yhist, 1), cur_yhist);
        title("Current");
        subplot(3, 1, 3);
        plot(y_lags, y_xcorr);
        title("XCorr")
      end

      # TODO: find several hypotheses?
      [_ x_max_lag_i] = max(x_xcorr);
      [_ y_max_lag_i] = max(y_xcorr);
      d_x_dir = x_lags(x_max_lag_i) * this.chist_res;
      d_y_dir = y_lags(y_max_lag_i) * this.chist_res;
      # TODO: rotation
      main_dir = 0;
      rt = [cos(main_dir) -sin(main_dir); sin(main_dir) cos(main_dir)];
      d_pose(1:2) = rt * [d_x_dir; d_y_dir];

    endfunction
      
  endmethods
endclassdef
