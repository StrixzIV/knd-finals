function workspace_sim()
%% SCARA RPRR — Workspace & Reach Simulation
% Joint limits and step resolutions referenced from teleop.py:
%   M1 (J1 Base)  : 1000 steps = 45 deg,  limit ±80 deg from home
%   M2 (J2 Z)     : 1000 steps = 2  cm,   limit  0–13 cm (0.00–0.13 m)
%   M3 (J3 Elbow) : 1000 steps = 60 deg,  limit ±80 deg from home
%   M4 (J4 Wrist) : 100  steps = 15 deg,  limit ±80 deg from home
%
% Home pose: arm pointed straight out along +Y axis.
% The +90° offset on J1 is identical to the teleop FK (th1 + pi/2).

clear; clc; close all;

%% ═══════════════════════════════════════════════════════════════
%  ROBOT PARAMETERS  (DH / link lengths from teleop.py compute_fk)
%% ═══════════════════════════════════════════════════════════════
L1 = 0.24;       % Link 1 (a3 in DH table, Joint 3 row)  (m)
L2 = 0.147;      % Link 2 (a4 in DH table, Joint 4 row)  (m)
d1 = 0.08425;    % Fixed base height (d1 in DH table)     (m)

% ── Joint limits from teleop.py MOTORS dict ──────────────────────
J1_lim =  80.0;    % M1: ±80 deg  (revolute, symmetric about home)
J3_lim =  80.0;    % M3: ±80 deg  (revolute, symmetric about home)
J4_lim =  80.0;    % M4: ±80 deg  (not used for workspace XYZ, but noted)

d2_min = 0.00;     % M2 lower limit: 0 cm → 0.00 m
d2_max = 0.13;     % M2 upper limit: 13 cm → 0.13 m  (from teleop limit 13.0 cm)

% ── Step-to-unit calibration (informational, shown in figure titles) ──
% M1: 1000 steps / 45 deg  → 22.22 steps/deg
% M2: 1000 steps / 2  cm   → 500   steps/cm
% M3: 1000 steps / 60 deg  → 16.67 steps/deg
% M4: 100  steps / 15 deg  →  6.67 steps/deg

% ── Derived workspace extents ────────────────────────────────────
r_max = L1 + L2;        % Fully extended reach  (m)
r_min = abs(L1 - L2);   % Fully folded  reach   (m)

% Home direction offset: physical 0 deg points along +Y (not +X).
% Identical to teleop: th1_world = deg2rad(J1_pos) + pi/2
home_offset_deg = 90.0;   % degrees added to J1 before FK

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 1 — 2D Top-Down Reachable Ring  (constrained J1 & J3)
%% ═══════════════════════════════════════════════════════════════
fig1 = figure('Name', 'SCARA Workspace — Top View (constrained)', ...
              'Position', [50 350 560 520], 'Color', 'w');
ax1  = axes(fig1, 'Position', [0.1 0.1 0.8 0.8]);
hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on');
ax_lim = r_max * 1.25;
ax1.XLim = [-ax_lim ax_lim]; ax1.YLim = [-ax_lim ax_lim];
ax1.XLabel.String = 'X (m)'; ax1.YLabel.String = 'Y (m)';
title(ax1, sprintf('Reachable workspace — top-down  |  J1: ±%.0f°,  J3: ±%.0f°', ...
      J1_lim, J3_lim));

% Full annulus outlines for reference (grey, dashed)
theta_ring = linspace(0, 2*pi, 360);
plot(ax1, r_max*cos(theta_ring), r_max*sin(theta_ring), ...
     'Color', [0.75 0.75 0.75], 'LineStyle', '--', 'LineWidth', 1.0);
plot(ax1, r_min*cos(theta_ring), r_min*sin(theta_ring), ...
     'Color', [0.75 0.75 0.75], 'LineStyle', '--', 'LineWidth', 0.8);

% ── Constrained EE scatter (respecting real joint limits) ─────────
N_samples = 4000;
% J1 joint position drawn uniformly in [-J1_lim, +J1_lim] deg
% World angle = J1_pos + home_offset_deg
j1_rnd   = (rand(N_samples,1)*2 - 1) * J1_lim;          % joint space
t1_world = j1_rnd + home_offset_deg;                      % world frame
j3_rnd   = (rand(N_samples,1)*2 - 1) * J3_lim;           % joint space
ex_rnd   = L1*cosd(t1_world) + L2*cosd(t1_world + j3_rnd);
ey_rnd   = L1*sind(t1_world) + L2*sind(t1_world + j3_rnd);

% Convex hull shading
try
    k_ch = convhull(ex_rnd, ey_rnd);
    fill(ax1, ex_rnd(k_ch), ey_rnd(k_ch), ...
         [0.82 0.90 0.97], 'EdgeColor', [0.3 0.5 0.8], ...
         'LineWidth', 1.2, 'FaceAlpha', 0.55);
catch; end

scatter(ax1, ex_rnd, ey_rnd, 3, [0.15 0.40 0.75], ...
        'filled', 'MarkerFaceAlpha', 0.20);

% Radius annotation lines
text(ax1, r_max+0.01, 0, sprintf('r_{max} = %.3f m', r_max), ...
     'FontSize', 8, 'Color', [0.1 0.3 0.7]);
text(ax1, r_min+0.01, 0, sprintf('r_{min} = %.3f m', r_min), ...
     'FontSize', 8, 'Color', [0.1 0.3 0.7]);

% Home direction arrow (+Y)
quiver(ax1, 0, 0, 0, r_max*0.85, 0, 'g', 'LineWidth', 2.0, ...
       'MaxHeadSize', 0.15);
text(ax1, 0.02, r_max*0.88, 'Home (+Y)', 'FontSize', 8, 'Color', [0 0.55 0]);

% J1 arc limits from home
arc_ang  = linspace(home_offset_deg - J1_lim, home_offset_deg + J1_lim, 120);
arc_r    = r_max * 0.96;
plot(ax1, arc_r*cosd(arc_ang), arc_r*sind(arc_ang), 'r-', 'LineWidth', 1.5);
% Limit rays
for sgn = [-1, 1]
    ang_lim = home_offset_deg + sgn*J1_lim;
    plot(ax1, [0, r_max*cosd(ang_lim)], [0, r_max*sind(ang_lim)], ...
         'r--', 'LineWidth', 1.0);
    text(ax1, (r_max+0.03)*cosd(ang_lim), (r_max+0.03)*sind(ang_lim), ...
         sprintf('%+.0f°', sgn*J1_lim), 'FontSize', 8, 'Color', [0.8 0 0]);
end

% Origin
plot(ax1, 0, 0, 'k+', 'MarkerSize', 12, 'LineWidth', 2);

legend(ax1, {'r_{max} ref', 'r_{min} ref', 'Reachable hull', 'EE samples', ...
             'Home dir', 'J1 arc', 'J1 limits'}, ...
       'Location', 'southeast', 'FontSize', 8);

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 2 — 3D Workspace Volume  (all d₂ heights, constrained)
%% ═══════════════════════════════════════════════════════════════
fig2 = figure('Name', 'SCARA Workspace — 3D Volume (constrained)', ...
              'Position', [620 350 580 520], 'Color', 'w');
ax2  = axes(fig2);
hold(ax2, 'on'); grid(ax2, 'on'); view(ax2, 35, 25);
ax2.XLabel.String = 'X (m)';
ax2.YLabel.String = 'Y (m)';
ax2.ZLabel.String = 'Z (m)';
title(ax2, sprintf('3D reachable volume  |  d₂: %.0f–%.0f cm,  J1: ±%.0f°,  J3: ±%.0f°', ...
      d2_min*100, d2_max*100, J1_lim, J3_lim));

N3    = 6000;
j1_3  = (rand(N3,1)*2 - 1) * J1_lim;
t1_w3 = j1_3 + home_offset_deg;
j3_3  = (rand(N3,1)*2 - 1) * J3_lim;
d2_3  = rand(N3,1) * d2_max;       % 0 to d2_max metres

ex_3  = L1*cosd(t1_w3) + L2*cosd(t1_w3 + j3_3);
ey_3  = L1*sind(t1_w3) + L2*sind(t1_w3 + j3_3);
ez_3  = d1 + d2_3;

scatter3(ax2, ex_3, ey_3, ez_3, 5, ez_3, 'filled', 'MarkerFaceAlpha', 0.25);
colormap(ax2, 'cool');
cb = colorbar(ax2);
cb.Label.String = 'Z height (m)';

% Bounding cylinder outlines at floor and ceiling
z_levels = [d1 + d2_min, d1 + d2_max];
% Use constrained arc for J1
arc_full  = linspace(home_offset_deg - J1_lim, home_offset_deg + J1_lim, 180);
% Outer arc at r_max
xo_arc = r_max * cosd(arc_full);
yo_arc = r_max * sind(arc_full);
% Inner arc at r_min
xi_arc = r_min * cosd(arc_full);
yi_arc = r_min * sind(arc_full);

for zl = z_levels
    plot3(ax2, xo_arc, yo_arc, zl*ones(size(xo_arc)), 'b-',  'LineWidth', 1.2);
    plot3(ax2, xi_arc, yi_arc, zl*ones(size(xi_arc)),  'b--', 'LineWidth', 0.8);
end
% Vertical edges at J1 limits
for sgn_v = [-1, 1]
    ang_v = home_offset_deg + sgn_v * J1_lim;
    for rad_v = [r_min, r_max]
        xv = rad_v*cosd(ang_v);
        yv = rad_v*sind(ang_v);
        plot3(ax2, [xv xv], [yv yv], z_levels, 'b:', 'LineWidth', 0.8);
    end
end

% Home column
plot3(ax2, [0 0], [0 0], [0, d1+d2_max+0.02], 'k-', 'LineWidth', 2);
plot3(ax2, 0, 0, 0, 'k.', 'MarkerSize', 14);
ax2.ZLim = [0, d1+d2_max+0.03];

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 3 — Interactive Cross-Section Slicer
%% ═══════════════════════════════════════════════════════════════
fig3 = uifigure('Name', 'SCARA Workspace — Interactive Slicer', ...
                'Position', [50 30 820 320]);

ax3 = uiaxes(fig3, 'Position', [250 10 555 295]);
hold(ax3, 'on'); axis(ax3, 'equal'); grid(ax3, 'on');
ax3.XLim = [-ax_lim ax_lim]; ax3.YLim = [-ax_lim ax_lim];
ax3.XLabel.String = 'X (m)'; ax3.YLabel.String = 'Y (m)';

% ── Controls ──────────────────────────────────────────────────────
uilabel(fig3, 'Position', [10 285 230 18], ...
    'Text', 'd₂ height slice (0–13 cm)', 'FontWeight', 'bold');
sld_d2s = uislider(fig3, 'Position', [10 275 220 3], ...
    'Limits', [d2_min d2_max], 'Value', d2_max/2, ...
    'ValueChangingFcn', @(s,e) updateSlice(e.Value));
lbl_d2s = uilabel(fig3, 'Position', [160 285 80 18], ...
    'Text', sprintf('%.2f m', d2_max/2), 'HorizontalAlignment', 'right');

uilabel(fig3, 'Position', [10 240 230 18], ...
    'Text', 'Elbow solution', 'FontWeight', 'bold');
elbow_btn = uibutton(fig3, 'state', ...
    'Position', [10 215 220 24], ...
    'Text', 'Elbow-up (+J3)', ...
    'ValueChangedFcn', @(b,e) updateSlice(sld_d2s.Value));

uilabel(fig3, 'Position', [10 185 230 18], ...
    'Text', sprintf('J1 sweep (±%.0f° max)', J1_lim), 'FontWeight', 'bold');
sld_t1s = uislider(fig3, 'Position', [10 175 220 3], ...
    'Limits', [5 J1_lim], 'Value', J1_lim, ...
    'ValueChangingFcn', @(s,e) updateSlice(sld_d2s.Value));
lbl_t1s = uilabel(fig3, 'Position', [160 185 80 18], ...
    'Text', sprintf('±%.0f°', J1_lim), 'HorizontalAlignment', 'right');

uilabel(fig3, 'Position', [10 140 230 18], ...
    'Text', sprintf('J3 range (±%.0f° max)', J3_lim), 'FontWeight', 'bold');
sld_j3s = uislider(fig3, 'Position', [10 130 220 3], ...
    'Limits', [5 J3_lim], 'Value', J3_lim, ...
    'ValueChangingFcn', @(s,e) updateSlice(sld_d2s.Value));
lbl_j3s = uilabel(fig3, 'Position', [160 140 80 18], ...
    'Text', sprintf('±%.0f°', J3_lim), 'HorizontalAlignment', 'right');

lbl_info = uilabel(fig3, 'Position', [10 20 230 100], ...
    'Text', '', 'FontSize', 9, 'WordWrap', 'on');

% Initial draw
updateSlice(d2_max / 2);

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 4 — Reach vs J3 Elbow Angle  (within ±J3_lim)
%% ═══════════════════════════════════════════════════════════════
fig4 = figure('Name', 'SCARA Reach vs J3 (Elbow)', ...
              'Position', [620 30 580 290], 'Color', 'w');
ax4  = axes(fig4);
t3_sweep = linspace(-J3_lim, J3_lim, 500);
reach    = sqrt((L1 + L2*cosd(t3_sweep)).^2 + (L2*sind(t3_sweep)).^2);
plot(ax4, t3_sweep, reach, 'b-', 'LineWidth', 2);
hold(ax4, 'on');
yline(ax4, r_max, 'r--', 'LineWidth', 1.2, ...
      'Label', sprintf('r_{max} = %.3f m', r_max), 'LabelHorizontalAlignment', 'left');
yline(ax4, r_min, 'g--', 'LineWidth', 1.2, ...
      'Label', sprintf('r_{min} = %.3f m', r_min), 'LabelHorizontalAlignment', 'left');
xline(ax4,  J3_lim, 'm:', 'LineWidth', 1.0, 'Label', sprintf('+%.0f° limit', J3_lim));
xline(ax4, -J3_lim, 'm:', 'LineWidth', 1.0, 'Label', sprintf('−%.0f° limit', J3_lim));

% Mark actual reach at limits
reach_at_lim = sqrt((L1 + L2*cosd(J3_lim))^2 + (L2*sind(J3_lim))^2);
scatter(ax4, [ J3_lim, -J3_lim], [reach_at_lim, reach_at_lim], 60, 'm', 'filled');
text(ax4, J3_lim+1, reach_at_lim+0.005, sprintf('%.3f m', reach_at_lim), ...
     'FontSize', 8, 'Color', [0.6 0 0.6]);

ax4.XLabel.String = 'J3 elbow angle (°)';
ax4.YLabel.String = 'Reach from base (m)';
title(ax4, sprintf('Reach vs J3 elbow angle  (limit: ±%.0f°,  1000 steps = 60°)', J3_lim));
grid(ax4, 'on');
ax4.XLim = [-J3_lim*1.2, J3_lim*1.2];

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 5 — Step Resolution Reference Table
%% ═══════════════════════════════════════════════════════════════
fig5 = figure('Name', 'Motor Step Resolution', ...
              'Position', [50 30 420 200], 'Color', 'w', ...
              'MenuBar', 'none', 'ToolBar', 'none');
ax5 = axes(fig5, 'Visible', 'off', 'Position', [0 0 1 1]);
col_hdr  = {'Motor', 'Joint', 'Steps/unit', 'Unit', 'Limit'};
col_data = {
    'M1',  'J1 Base',  sprintf('%.2f steps/°',  1000/45),  'deg',  '±80°';
    'M2',  'J2 Z',     sprintf('%.1f steps/cm', 1000/2),   'cm',   '0-13 cm';
    'M3',  'J3 Elbow', sprintf('%.2f steps/°',  1000/60),  'deg',  '±80°';
    'M4',  'J4 Wrist', sprintf('%.2f steps/°',  100/15),   'deg',  '±80°';
};
tbl_data = [col_hdr; col_data];
t = uitable(fig5, ...
    'Data', col_data, ...
    'ColumnName', col_hdr, ...
    'ColumnWidth', {45, 80, 110, 40, 70}, ...
    'RowName', {}, ...
    'Position', [10 10 400 150], ...
    'FontSize', 10);
title(ax5, 'Step Resolution (from teleop.py calibration)', ...
      'Units', 'normalized', 'Position', [0.5 0.97]);

%% ═══════════════════════════════════════════════════════════════
%  SLICER CALLBACK
%% ═══════════════════════════════════════════════════════════════

    function updateSlice(d2_val)
        lbl_d2s.Text = sprintf('%.3f m', d2_val);
        z_slice = d1 + d2_val;

        j1_sweep = sld_t1s.Value;   % half-range, degrees
        lbl_t1s.Text = sprintf('±%.0f°', j1_sweep);

        j3_range = sld_j3s.Value;   % half-range, degrees
        lbl_j3s.Text = sprintf('±%.0f°', j3_range);

        sign_t3 = 1;
        if elbow_btn.Value
            sign_t3 = -1;
            elbow_btn.Text = 'Elbow-down (−J3)';
        else
            elbow_btn.Text = 'Elbow-up (+J3)';
        end

        % ── Boundary traces ──────────────────────────────────────
        N_theta = 720;
        j1_arr  = linspace(-j1_sweep, j1_sweep, N_theta);
        t1_w    = j1_arr + home_offset_deg;

        ex_outer = zeros(1, N_theta);
        ey_outer = zeros(1, N_theta);
        ex_inner = zeros(1, N_theta);
        ey_inner = zeros(1, N_theta);

        for k = 1:N_theta
            th1 = t1_w(k);
            % Outer boundary at max J3 (most extended)
            t3_out = sign_t3 * j3_range;
            ex_outer(k) = L1*cosd(th1) + L2*cosd(th1 + t3_out);
            ey_outer(k) = L1*sind(th1) + L2*sind(th1 + t3_out);
            % Inner boundary at min J3 (most folded within limit)
            t3_in = sign_t3 * (-j3_range);
            ex_inner(k) = L1*cosd(th1) + L2*cosd(th1 + t3_in);
            ey_inner(k) = L1*sind(th1) + L2*sind(th1 + t3_in);
        end

        % ── Random scatter within real limits ─────────────────────
        N_s  = 2000;
        j1_s = (rand(N_s,1)*2 - 1) * j1_sweep;
        t1_s = j1_s + home_offset_deg;
        j3_s = sign_t3 * rand(N_s,1) * j3_range;
        ex_s = L1*cosd(t1_s) + L2*cosd(t1_s + j3_s);
        ey_s = L1*sind(t1_s) + L2*sind(t1_s + j3_s);

        cla(ax3);
        hold(ax3, 'on');

        % Convex hull fill
        try
            all_pts = [ex_s; ex_outer'; ex_inner'];
            all_py  = [ey_s; ey_outer'; ey_inner'];
            k_ch = convhull(all_pts, all_py);
            fill(ax3, all_pts(k_ch), all_py(k_ch), ...
                 [0.82 0.90 0.97], 'EdgeColor', 'none', 'FaceAlpha', 0.55);
        catch; end

        scatter(ax3, ex_s, ey_s, 3, [0.2 0.45 0.75], ...
                'filled', 'MarkerFaceAlpha', 0.25);

        % Boundary traces
        plot(ax3, ex_outer, ey_outer, 'b-',  'LineWidth', 2.0);
        plot(ax3, ex_inner, ey_inner, 'b--', 'LineWidth', 1.2);

        % Home direction indicator
        quiver(ax3, 0, 0, 0, r_max*0.5, 0, 'g', 'LineWidth', 1.8, 'MaxHeadSize', 0.2);

        % J1 limit rays from origin (in world frame)
        for sgn_r = [-1, 1]
            ang_r = home_offset_deg + sgn_r * j1_sweep;
            plot(ax3, [0, r_max*cosd(ang_r)], [0, r_max*sind(ang_r)], ...
                 'r--', 'LineWidth', 0.9);
        end

        % Origin
        plot(ax3, 0, 0, 'k+', 'MarkerSize', 12, 'LineWidth', 2);

        title(ax3, sprintf('XY slice  d₂ = %.3f m (Z = %.3f m) | J1: ±%.0f°  J3: ±%.0f°', ...
              d2_val, z_slice, j1_sweep, j3_range));
        ax3.XLim = [-ax_lim ax_lim];
        ax3.YLim = [-ax_lim ax_lim];

        % Stats info panel
        r_pts   = sqrt(ex_s.^2 + ey_s.^2);
        area_est = pi * (r_max^2 - r_min^2) * (2*j1_sweep/360);
        lbl_info.Text = sprintf( ...
            ['Z = %.3f m\n' ...
             'J1 sweep: ±%.0f°\n' ...
             'J3 range: ±%.0f°\n' ...
             'r: %.3f – %.3f m\n' ...
             'Sector area ≈ %.4f m²\n' ...
             'd₂ range: 0–13 cm'], ...
            z_slice, j1_sweep, j3_range, r_min, r_max, area_est);
    end

end  % workspace_sim