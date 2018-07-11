addpath(genpath(pwd));
load_packages();
close all;
clear;

# Params

global scan_matcher = OrigAhsm("ahist_res", deg2rad(1), "chist_res", 0.25);

figure("name", "Current Test",  "numbertitle", "off",
       "menubar", "none", "toolbar", "none");

################################################################################
# Test Suite

global failed_tests_nm = 0;
global total_tests_nm = 0;

function poses = range_to_poses(params_mtx)
  poses = [];
  for th = params_mtx(3, 1):params_mtx(3, 2):params_mtx(3, 3)
    for x = params_mtx(1, 1):params_mtx(1, 2):params_mtx(1, 3)
      for y = params_mtx(2, 1):params_mtx(2, 2):params_mtx(2, 3)
        poses(size(poses, 1) + 1, :) = [x y th];
      endfor
    endfor
  endfor
endfunction

function is_eq = is_float_eq(a, b)
  is_eq = abs(a - b) < 1e-4;
endfunction

function is_eq = is_pose_eq(p1, p2)
  is_eq = and(is_float_eq(p1(1), p2(1)),
              is_float_eq(p1(2), p2(2)),
              is_float_eq(p1(3), p2(3)));
endfunction

function test_scan_matching(world_id, world, ref_pose, transform)
  global total_tests_nm;
  total_tests_nm += 1;

  global scan_matcher;
  sensor = setup_sensor();
  cur_pose = ref_pose + transform;

  ref_scan = get_scan_view(ref_pose, sensor, world);
  cur_scan = get_scan_view(cur_pose, sensor, world);

  est_transform = scan_matcher.find_transformation(ref_scan, ref_pose,
                                                   cur_scan, ref_pose);
  is_ok = is_pose_eq(est_transform, transform);

  f = gcf();
  if (f)
    if (is_ok)
      status = "OK";
    else
      status = "Err";
    endif
    title = sprintf("[%s] %s + %s = %s", world_id,
                    pose2str(ref_pose), pose2str(transform), status);
    display_env2D(world, ref_scan, ref_pose, cur_scan,
                  ref_pose + est_transform, ref_pose + transform, f);
  endif

  if (is_ok)
    return; # Ok
  endif

  global failed_tests_nm;
  failed_tests_nm += 1;
  fprintf("[%s] Pose:%s; Transform:%s; Estimated:%s.\n", world_id,
          pose2str(ref_pose), pose2str(transform), pose2str(est_transform));
endfunction

function test_world(world_id, w, ref_poses, trans_poses)
  for pose_i = 1:size(ref_poses, 1)
    pose = ref_poses(pose_i, :);
    for trans_i = 1:size(trans_poses, 1)
      trans = trans_poses(trans_i, :);
      test_scan_matching(world_id, w, pose, trans);
    endfor
  endfor
endfunction

################################################################################
### Test cases

## Smoke

#ref_poses = range_to_poses([-7 1 -6.5; ...
#                            -7 1 -6.5; ...
#                            -pi deg2rad(10) -pi + 0.00001]);
ref_poses = [-5 4 deg2rad(5)];#;
             ## 8 -6 deg2rad(-5);
             ## -7 -7 deg2rad(225);
             ## -5 4 deg2rad(5);
             ## 7 3 deg2rad(20)];
trans_poses = range_to_poses([-1 7 1; -1 7 1; -pi/6 deg2rad(36) pi/6]);

test_world("Room1", SegmentedWorld.sq_room(), ref_poses, trans_poses);
test_world("Corridor1", SegmentedWorld.corridor(), ref_poses, trans_poses);
test_world("Room2", SegmentedWorld.free_corners(), ref_poses, trans_poses);

fprintf("== DONE ==\n");
if (failed_tests_nm == 0)
  fprintf("All tests are passed.\n");
else
  fprintf("%d of %d failed.\n", failed_tests_nm, total_tests_nm);
endif
