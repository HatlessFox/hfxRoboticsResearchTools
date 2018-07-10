addpath(genpath(pwd))

#pkg install -forge <pkg-name>
pkg load statistics # chi2inv, etc.
pkg load octclip # oc_polybool
pkg load geometry # intersectPolylines, etc.
pkg load signal # ahsm
pkg load ltfat # ahsm

close all
clear


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


ww = wall_world();
world = init_world(ww);
sensor = setup_sensor();
# TODO: createMap

# -delta_ang -> +90 error
ref_pose = [0 0 deg2rad(36)];
cur_pose = [1 1 deg2rad(30)];

ref_scan = get_scan_view(ref_pose, sensor, world);
cur_scan = get_scan_view(cur_pose, sensor, world);

# filterScan(..) ?

d_pose = [0 0 0];
d_pose = scan_matcher.find_transformation(ref_scan, ref_pose,
                                          cur_scan, ref_pose);

est_pose = ref_pose + d_pose;

fprintf('== DONE ==\n')
fprintf('[GT Pose] x: %.2f; y: %.2f; th: %.2f.\n',
        cur_pose(1), cur_pose(2), rad2deg(cur_pose(3)));
fprintf('[SM Pose] x: %.2f; y: %.2f; th: %.2f.\n',
        est_pose(1), est_pose(2), rad2deg(est_pose(3)));

#------------------------------------------------------------------------------#
# Plotting

# blue - ref
# green - groud truth (curr)
# red - corrected

# draw map
f = figure;
axis equal;

plot(ww.walls(1,:), ww.walls(2,:), 'LineWidth', 1, 'color', 'k');
hold on;
grid on;
# TODO: 8-6-24 robot display artifact
display_robot2D(ref_scan, ref_pose, 'b');
display_robot2D(cur_scan, est_pose, 'r');
display_robot2D(cur_scan, cur_pose, 'g');
# TODO: show pose values as text

ginput(1);
#input("press to continue");

