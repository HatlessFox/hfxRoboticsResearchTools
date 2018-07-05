function [hst cbins] = make_histogram(seq, min_e, max_e, step)
  ## bin_nm = ceil((max_e - min_e) / step);
  ## hst = zeros(bin_nm, 1);
  ## for e = seq
  ##   #assert(min <= e && e <= max);
  ##   bin_i = floor((e - min_e) / step) + 1;
  ##   #[deg2rad(e) bin_i hst(bin_i)]
  ##   ## if (e == max_e)
  ##   ##    bin_i = bin_nm;
  ##   ## endif
  ##   hst(bin_i) = hst(bin_i) + 1;
  ## endfor
  [hst cbins] = hist(seq, min_e:step:max_e);
  hst = normalize(hst, "peak");
endfunction
