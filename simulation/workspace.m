function workspace_sim()
%% SCARA RPRR — Workspace & Reach Simulation
% Shows: XY reachable ring, 3D workspace volume, and an interactive
% cross-section slicer with sliders for d2 height and elbow solution.

clear; clc; close all;

%% Robot parameters — match your simulation.m values
L1 = 0.245;   % Link 1 length (m)
L2 = 0.155;   % Link 2 length (m)
d1 = 0.1075;   % Fixed base height (m)
d2_min = 0.0; % Prismatic min (m)
d2_max = 0.25; % Prismatic max (m)

r_max = L1 + L2;          % 0.70 m  — fully extended
r_min = abs(L1 - L2);     % 0.10 m  — fully folded

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 1 — 2D Top-Down Reachable Ring
%% ═══════════════════════════════════════════════════════════════
fig1 = figure('Name', 'SCARA Workspace — Top View', ...
              'Position', [50 350 560 520], 'Color', 'w');
ax1  = axes(fig1, 'Position', [0.1 0.1 0.8 0.8]);
hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on');
ax1.XLim = [-0.85 0.85]; ax1.YLim = [-0.85 0.85];
ax1.XLabel.String = 'X (m)'; ax1.YLabel.String = 'Y (m)';
title(ax1, 'Reachable workspace — top-down (XY plane)');

% Shaded annulus
theta_ring = linspace(0, 2*pi, 360);
x_out = r_max * cos(theta_ring);
y_out = r_max * sin(theta_ring);
x_in  = r_min * cos(theta_ring);
y_in  = r_min * sin(theta_ring);

% Fill outer circle
fill(ax1, x_out, y_out, [0.82 0.90 0.97], 'EdgeColor', 'none');
% Cut out inner dead zone
fill(ax1, x_in,  y_in,  [1 1 1],          'EdgeColor', 'none');
% Border circles
plot(ax1, x_out, y_out, 'b-',  'LineWidth', 1.5);
plot(ax1, x_in,  y_in,  'b--', 'LineWidth', 1.0);

% Radius annotations
text(ax1, r_max+0.02, 0, sprintf('r_{max} = %.2f m', r_max), ...
     'FontSize', 9, 'Color', [0.1 0.3 0.7]);
text(ax1, r_min+0.02, 0, sprintf('r_{min} = %.2f m', r_min), ...
     'FontSize', 9, 'Color', [0.1 0.3 0.7]);

% Sample random poses and scatter EE points
N_samples = 2000;
t1_rnd = rand(N_samples,1)*360 - 180;
t3_rnd = rand(N_samples,1)*340 - 170;
ex_rnd = L1*cosd(t1_rnd) + L2*cosd(t1_rnd + t3_rnd);
ey_rnd = L1*sind(t1_rnd) + L2*sind(t1_rnd + t3_rnd);
scatter(ax1, ex_rnd, ey_rnd, 2, [0.2 0.5 0.8], 'filled', 'MarkerFaceAlpha', 0.25);

% Origin + axes
plot(ax1, 0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2);
plot(ax1, [0 r_max], [0 0], 'k--', 'LineWidth', 0.8);
plot(ax1, [0 0], [0 r_max], 'k--', 'LineWidth', 0.8);

legend(ax1, {'Reachable area', 'Dead zone', 'r_{max}', 'r_{min}', 'EE samples'}, ...
       'Location', 'northeast', 'FontSize', 8);

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 2 — 3D Workspace Volume
%% ═══════════════════════════════════════════════════════════════
fig2 = figure('Name', 'SCARA Workspace — 3D Volume', ...
              'Position', [620 350 580 520], 'Color', 'w');
ax2  = axes(fig2);
hold(ax2, 'on'); grid(ax2, 'on'); view(ax2, 35, 25);
ax2.XLabel.String = 'X (m)';
ax2.YLabel.String = 'Y (m)';
ax2.ZLabel.String = 'Z (m)';
title(ax2, '3D reachable volume (all d₂ heights)');

% Generate 3D cloud
N3 = 4000;
t1_3  = rand(N3,1)*360 - 180;
t3_3  = rand(N3,1)*340 - 170;
d2_3  = rand(N3,1)*d2_max;
ex_3  = L1*cosd(t1_3) + L2*cosd(t1_3 + t3_3);
ey_3  = L1*sind(t1_3) + L2*sind(t1_3 + t3_3);
ez_3  = d1 + d2_3;

scatter3(ax2, ex_3, ey_3, ez_3, 4, ez_3, 'filled', 'MarkerFaceAlpha', 0.3);
colormap(ax2, 'cool');
cb = colorbar(ax2);
cb.Label.String = 'Z height (m)';

% Draw bounding cylinder outline
z_levels = [d1+d2_min, d1+d2_max];
for zl = z_levels
    plot3(ax2, x_out, y_out, zl*ones(size(x_out)), 'b-',  'LineWidth', 1.2);
    plot3(ax2, x_in,  y_in,  zl*ones(size(x_in)),  'b--', 'LineWidth', 0.8);
end
% Vertical edges at r_max
for ang = 0:90:270
    xv = r_max*cosd(ang); yv = r_max*sind(ang);
    plot3(ax2, [xv xv], [yv yv], z_levels, 'b-', 'LineWidth', 0.8);
end

% Origin column
plot3(ax2, [0 0], [0 0], [0 d1+d2_max+0.05], 'k-', 'LineWidth', 2);
plot3(ax2, 0, 0, 0, 'k.', 'MarkerSize', 14);

ax2.ZLim = [0 d1+d2_max+0.05];

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 3 — Interactive Cross-Section Slicer
%% ═══════════════════════════════════════════════════════════════
fig3 = uifigure('Name', 'SCARA Workspace — Interactive Slicer', ...
                'Position', [50 30 760 300]);

ax3 = uiaxes(fig3, 'Position', [220 10 520 275]);
hold(ax3, 'on'); axis(ax3, 'equal'); grid(ax3, 'on');
ax3.XLim = [-0.85 0.85]; ax3.YLim = [-0.85 0.85];
ax3.XLabel.String = 'X (m)'; ax3.YLabel.String = 'Y (m)';

% Controls
uilabel(fig3, 'Position', [10 255 200 20], ...
    'Text', 'd₂ height slice (m)', 'FontWeight', 'bold');
sld_d2s = uislider(fig3, 'Position', [10 245 190 3], ...
    'Limits', [d2_min d2_max], 'Value', 0.1, ...
    'ValueChangingFcn', @(s,e) updateSlice(e.Value));
lbl_d2s = uilabel(fig3, 'Position', [140 255 70 20], ...
    'Text', '0.10 m', 'HorizontalAlignment', 'right');

uilabel(fig3, 'Position', [10 205 200 20], ...
    'Text', 'Elbow solution', 'FontWeight', 'bold');
elbow_btn = uibutton(fig3, 'state', ...
    'Position', [10 178 190 24], ...
    'Text', 'Elbow-up', ...
    'ValueChangedFcn', @(b,e) updateSlice(sld_d2s.Value));

uilabel(fig3, 'Position', [10 148 200 20], ...
    'Text', 'θ₁ sweep (°)', 'FontWeight', 'bold');
sld_t1s = uislider(fig3, 'Position', [10 138 190 3], ...
    'Limits', [10 360], 'Value', 360, ...
    'ValueChangingFcn', @(s,e) updateSlice(sld_d2s.Value));
lbl_t1s = uilabel(fig3, 'Position', [140 148 70 20], ...
    'Text', '360°', 'HorizontalAlignment', 'right');

lbl_info = uilabel(fig3, 'Position', [10 50 200 85], ...
    'Text', '', 'FontSize', 10, 'WordWrap', 'on');

% Initial draw
updateSlice(0.1);

%% ═══════════════════════════════════════════════════════════════
%  FIGURE 4 — Reach vs Joint Angle Plot
%% ═══════════════════════════════════════════════════════════════
fig4 = figure('Name', 'SCARA Reach vs θ₃', ...
              'Position', [620 30 580 280], 'Color', 'w');
ax4  = axes(fig4);
t3_sweep = linspace(-170, 170, 500);
reach    = sqrt((L1 + L2*cosd(t3_sweep)).^2 + (L2*sind(t3_sweep)).^2);
plot(ax4, t3_sweep, reach, 'b-', 'LineWidth', 2);
hold(ax4, 'on');
yline(ax4, r_max, 'r--', 'LineWidth', 1, 'Label', sprintf('r_{max}=%.2fm', r_max));
yline(ax4, r_min, 'g--', 'LineWidth', 1, 'Label', sprintf('r_{min}=%.2fm', r_min));
ax4.XLabel.String = 'θ₃ elbow angle (°)';
ax4.YLabel.String = 'Reach from base (m)';
title(ax4, 'Reach as a function of θ₃ (elbow angle)');
grid(ax4, 'on');
ax4.XLim = [-170 170];

%% ═══════════════════════════════════════════════════════════════
%  SLICER CALLBACK
%% ═══════════════════════════════════════════════════════════════

    function updateSlice(d2_val)
        lbl_d2s.Text = sprintf('%.3f m', d2_val);
        z_slice = d1 + d2_val;

        sweep_deg = sld_t1s.Value;
        lbl_t1s.Text = sprintf('%.0f°', sweep_deg);

        sign_t3 = 1;
        if elbow_btn.Value
            sign_t3 = -1;
            elbow_btn.Text = 'Elbow-down';
        else
            elbow_btn.Text = 'Elbow-up';
        end

        % Dense reachable boundary at this d2 slice
        N_theta = 720;
        t1_arr = linspace(-sweep_deg/2, sweep_deg/2, N_theta);

        % Elbow-up and elbow-down traces at r_max and r_min
        ex_outer = zeros(1, N_theta);
        ey_outer = zeros(1, N_theta);
        ex_inner = zeros(1, N_theta);
        ey_inner = zeros(1, N_theta);

        for k = 1:N_theta
            th1 = t1_arr(k);
            % Outer boundary: t3 chosen to maximise reach
            t3_out = sign_t3 * acosd((r_max^2 - L1^2 - L2^2)/(2*L1*L2));
            if isreal(t3_out)
                ex_outer(k) = L1*cosd(th1) + L2*cosd(th1 + t3_out);
                ey_outer(k) = L1*sind(th1) + L2*sind(th1 + t3_out);
            end
            % Inner boundary: folded
            c3_in = (r_min^2 - L1^2 - L2^2)/(2*L1*L2);
            if abs(c3_in) <= 1
                t3_in = sign_t3 * acosd(c3_in);
                ex_inner(k) = L1*cosd(th1) + L2*cosd(th1 + t3_in);
                ey_inner(k) = L1*sind(th1) + L2*sind(th1 + t3_in);
            end
        end

        % Random sample at this height
        N_s = 1500;
        t1_s  = rand(N_s,1)*sweep_deg - sweep_deg/2;
        t3_s  = sign_t3 * rand(N_s,1)*340;
        ex_s  = L1*cosd(t1_s) + L2*cosd(t1_s + t3_s);
        ey_s  = L1*sind(t1_s) + L2*sind(t1_s + t3_s);

        cla(ax3);
        hold(ax3, 'on');

        % Filled workspace region (convex hull approach)
        try
            pts  = [ex_s, ey_s];
            k_ch = convhull(pts(:,1), pts(:,2));
            fill(ax3, pts(k_ch,1), pts(k_ch,2), ...
                 [0.82 0.90 0.97], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
        catch
        end

        scatter(ax3, ex_s, ey_s, 3, [0.2 0.45 0.75], ...
                'filled', 'MarkerFaceAlpha', 0.3);

        % Boundary traces
        plot(ax3, ex_outer, ey_outer, 'b-',  'LineWidth', 2);
        plot(ax3, ex_inner, ey_inner, 'b--', 'LineWidth', 1.2);

        % Origin
        plot(ax3, 0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2);

        title(ax3, sprintf('XY slice at d₂ = %.3f m  (Z = %.3f m)', d2_val, z_slice));
        ax3.XLim = [-0.85 0.85]; ax3.YLim = [-0.85 0.85];

        % Stats
        r_pts = sqrt(ex_s.^2 + ey_s.^2);
        lbl_info.Text = sprintf( ...
            'Z = %.3f m\nSweep: %.0f°\nr range: %.3f – %.3f m\nArea ≈ π(r_max²–r_min²) = %.4f m²', ...
            z_slice, sweep_deg, r_min, r_max, pi*(r_max^2 - r_min^2));
    end

end  % workspace_sim