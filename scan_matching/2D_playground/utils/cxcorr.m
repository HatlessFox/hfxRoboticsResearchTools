function [corr lags] = cxcorr(seq, ref_seq)
  # circular cross-correlation
  # a straightforward implementation  -> O(n^2)
  # NB: FFT-based saves computation   -> O(n*log(n)

  seq_nm = size(seq, 1);
  assert(seq_nm == size(ref_seq, 1));

  eff_ref = [ref_seq; ref_seq; ref_seq];
  eff_seq = [seq; zeros(seq_nm, 1)];
  [corr lags] = xcorr(eff_seq, eff_ref, seq_nm / 2);
endfunction
