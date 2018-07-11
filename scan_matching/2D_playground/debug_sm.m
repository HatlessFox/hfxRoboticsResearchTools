addpath(genpath(pwd));
load_packages();
close all;
clear;


# IDs are taken from https://github.com/szandara/2DScanMatching_SLAM

Opt.random.seed = sum(100*clock);
fprintf('Random seed: %6.0f.\n',Opt.random.seed);

# If using mixed algorithm 'icpbSM'
# you have to fill the next two with function handles.
#   Possible values: mahaAssociation (mahalanobis distance),
#                    mbAssociation (metric based ),
#                    cpAssociation (closest Point),
#                    p2lAssociation (point to line),
#                    normAssociation (normal lines association),
#                    mpAssociation (range based association).
#Opt.scanmatcher.associate = @mahaAssociation;
#   Possible Values: regist_besl (Besley Unit Vector),
#                    register_martinez (Closed form Least Square),
#                    register_matlab (MATLAB minimization),
#                    registerSVD (Singular Value Decomposition),
#                    registerCensi (Censi's Lagrange Multiplier).
#Opt.scanmatcher.register = @ga;
Opt.scanmatcher.rejection_rule = []; # rejection rule handling
Opt.scanmatcher.projfilter = 0; # use projection filter?
Opt.scanmatcher.iterations = 50; # max nm of iterations
Opt.scanmatcher.Br = [0.1 0.5]; # angular and radian tresholds
Opt.scanmatcher.map_res = 0.5; # map resolution
Opt.scanmatcher.convalue = 0.00001; # below the value the result is ok
Opt.scanmatcher.niterconv = 3; # min number of iters before convergence check
Opt.scanmatcher.chival = chi2inv(0.95, 0.1); # tolerance value (?)

Opt.scan.maxscanpoints = 1; # min num of pts before scan composition
Opt.error.display_result = 1;
Opt.map.resolution = Opt.scanmatcher.map_res;
Opt.plot.robot_scale = 1;

################################################################################
################################################################################
################################################################################

# Possible values: icp, IDC,
#   TODO: pIC,MbICP,ga,gmapping,montecarlo,fmtsm,houghSM,NDT,lf_sog, icpbSM.
#
# AngleHistogram: OrigAhsm

scan_matcher = OrigAhsm("debug", true);

world = SegmentedWorld.sq_room();
sensor = setup_sensor();
# TODO: createMap

ref_pose = [0 0 deg2rad(36)];
cur_pose = [1 1 deg2rad(30)];

ref_scan = get_scan_view(ref_pose, sensor, world);
cur_scan = get_scan_view(cur_pose, sensor, world);

# filterScan(..) ?

d_pose = [0 0 0];
d_pose = scan_matcher.find_transformation(ref_scan, ref_pose,
                                          cur_scan, ref_pose);

est_pose = ref_pose + d_pose;

fprintf("== DONE ==\n")
fprintf("[GT Pose] %s\n", pose2str(cur_pose));
fprintf("[SM Pose] %s\n", pose2str(est_pose));

display_env2D(world, ref_scan, ref_pose, cur_scan, est_pose, cur_pose);
ginput(1);
