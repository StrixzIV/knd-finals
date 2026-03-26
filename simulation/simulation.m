%% SCARA RPRR Interactive Simulator — FK / IK
% Joint limits, step calibration and DH parameters all match teleop.py:
%
%   M1 (J1 Base)  : 1000 steps = 45 deg,  joint limit ±80 deg
%   M2 (J2 Z)     : 1000 steps = 2  cm,   joint limit  0–13 cm (0–0.13 m)
%   M3 (J3 Elbow) : 1000 steps = 60 deg,  joint limit ±80 deg
%   M4 (J4 Wrist) : 100  steps = 15 deg,  joint limit ±80 deg
%
% Home pose: arm pointed straight along +Y axis.
% J1 world angle = J1_joint_deg + 90° (identical to teleop compute_fk: th1 + pi/2)
%
% DH table (from teleop.py compute_fk):
%   Row 1 — a=0,     alpha=0,    d=0.08425,  theta=th1          (J1 revolute)
%   Row 2 — a=0,     alpha=0,    d=d2,        theta=0            (J2 prismatic)
%   Row 3 — a=0.24,  alpha=pi,   d=0,         theta=th3          (J3 revolute)
%   Row 4 — a=0.147, alpha=0,    d=-0.06375,  theta=th4          (J4 revolute)
%   Row 5 — a=0,     alpha=0,    d=-0.1535,   theta=0            (EE offset)

function simulation()
clear; clc; close all;

%% ─── Robot parameters (DH / teleop.py) ──────────────────────────────
L1   = 0.24;      % Link 1 — DH a for J3 row  (m)
L2   = 0.147;     % Link 2 — DH a for J4 row  (m)
d1   = 0.08425;   % Fixed base column height   (m)

% ── Joint limits from teleop.py MOTORS dict ───────────────────────────
J1_MIN = -80;  J1_MAX =  80;   % degrees (joint space)
J2_MIN =  0.0; J2_MAX =  0.13; % metres  (0–13 cm)
J3_MIN = -80;  J3_MAX =  80;   % degrees
J4_MIN = -80;  J4_MAX =  80;   % degrees

HOME_OFFSET = 90.0;  % degrees — physical 0 deg → +Y axis

r_max = L1 + L2;
r_min = abs(L1 - L2);

%% ─── Figure ──────────────────────────────────────────────────────────
fig = uifigure('Name', 'SCARA RPRR Simulator', ...
               'Position', [100 100 960 680]);

%% ─── 3-D axes ────────────────────────────────────────────────────────
ax = uiaxes(fig, 'Position', [270 20 670 640]);
ax.XLim = [-0.5 0.5];
ax.YLim = [-0.5 0.5];
ax.ZLim = [0 0.3];
ax.XLabel.String = 'X (m)';
ax.YLabel.String = 'Y (m)';
ax.ZLabel.String = 'Z (m)';
grid(ax, 'on'); hold(ax, 'on'); view(ax, 45, 30);
title(ax, 'SCARA RPRR — FK Mode');

%% ─── Mode toggle ─────────────────────────────────────────────────────
uilabel(fig, 'Position', [10 638 245 22], ...
    'Text', 'Mode', 'FontWeight', 'bold', 'FontSize', 13);
mode_btn = uibutton(fig, 'state', ...
    'Position', [10 608 245 28], ...
    'Text', 'FK Mode  |  Switch to IK', ...
    'FontSize', 11, ...
    'ValueChangedFcn', @(btn,~) toggleMode(btn));

%% ─── FK Panel ────────────────────────────────────────────────────────
fk_panel = uipanel(fig, 'Title', 'Forward Kinematics', ...
    'Position', [10 295 250 305], 'FontWeight', 'bold');

% θ1  J1 Base
uilabel(fk_panel, 'Position', [10 250 180 20], ...
    'Text', sprintf('θ₁  Base (°)  [%+d … %+d]', J1_MIN, J1_MAX));
sld_t1 = uislider(fk_panel, 'Position', [10 238 210 3], ...
    'Limits', [J1_MIN J1_MAX], 'Value', 0, ...
    'ValueChangingFcn', @(~,e) updateFK(e.Value,[],[],[]));
lbl_t1 = uilabel(fk_panel, 'Position', [170 250 65 20], ...
    'Text', '0°', 'HorizontalAlignment', 'right');

% θ3  J3 Elbow
uilabel(fk_panel, 'Position', [10 195 180 20], ...
    'Text', sprintf('θ₃  Elbow (°)  [%+d … %+d]', J3_MIN, J3_MAX));
sld_t3 = uislider(fk_panel, 'Position', [10 183 210 3], ...
    'Limits', [J3_MIN J3_MAX], 'Value', -45, ...
    'ValueChangingFcn', @(~,e) updateFK([],e.Value,[],[]));
lbl_t3 = uilabel(fk_panel, 'Position', [170 195 65 20], ...
    'Text', '-45°', 'HorizontalAlignment', 'right');

% θ4  J4 Wrist
uilabel(fk_panel, 'Position', [10 140 180 20], ...
    'Text', sprintf('θ₄  Wrist (°)  [%+d … %+d]', J4_MIN, J4_MAX));
sld_t4 = uislider(fk_panel, 'Position', [10 128 210 3], ...
    'Limits', [J4_MIN J4_MAX], 'Value', 0, ...
    'ValueChangingFcn', @(~,e) updateFK([],[],e.Value,[]));
lbl_t4 = uilabel(fk_panel, 'Position', [170 140 65 20], ...
    'Text', '0°', 'HorizontalAlignment', 'right');

% d2  J2 Z-axis
uilabel(fk_panel, 'Position', [10 85 190 20], ...
    'Text', sprintf('d₂  Z-height (m)  [%.0f … %.0f cm]', ...
    J2_MIN*100, J2_MAX*100));
sld_d2 = uislider(fk_panel, 'Position', [10 73 210 3], ...
    'Limits', [J2_MIN J2_MAX], 'Value', 0.06, ...
    'ValueChangingFcn', @(~,e) updateFK([],[],[],e.Value));
lbl_d2 = uilabel(fk_panel, 'Position', [162 85 68 20], ...
    'Text', '0.060 m', 'HorizontalAlignment', 'right');

%% ─── IK Panel ────────────────────────────────────────────────────────
ik_panel = uipanel(fig, 'Title', 'Inverse Kinematics', ...
    'Position', [10 295 250 305], 'FontWeight', 'bold', 'Visible', 'off');

r_slide = r_max * 0.98;   % safe slider range (just inside max reach)

% X target
uilabel(ik_panel, 'Position', [10 250 200 20], ...
    'Text', sprintf('X target (m)  [±%.2f]', r_slide));
sld_x = uislider(ik_panel, 'Position', [10 238 210 3], ...
    'Limits', [-r_slide r_slide], 'Value', 0.25, ...
    'ValueChangingFcn', @(~,e) updateIK(e.Value,[],[],[]));
lbl_x = uilabel(ik_panel, 'Position', [162 250 68 20], ...
    'Text', '0.250 m', 'HorizontalAlignment', 'right');

% Y target
uilabel(ik_panel, 'Position', [10 195 200 20], ...
    'Text', sprintf('Y target (m)  [±%.2f]', r_slide));
sld_y = uislider(ik_panel, 'Position', [10 183 210 3], ...
    'Limits', [-r_slide r_slide], 'Value', 0.25, ...
    'ValueChangingFcn', @(~,e) updateIK([],e.Value,[],[]));
lbl_y = uilabel(ik_panel, 'Position', [162 195 68 20], ...
    'Text', '0.250 m', 'HorizontalAlignment', 'right');

% d2 height
uilabel(ik_panel, 'Position', [10 140 200 20], ...
    'Text', sprintf('d₂ height (m)  [0 … %.2f m]', J2_MAX));
sld_z = uislider(ik_panel, 'Position', [10 128 210 3], ...
    'Limits', [J2_MIN J2_MAX], 'Value', 0.06, ...
    'ValueChangingFcn', @(~,e) updateIK([],[],e.Value,[]));
lbl_z = uilabel(ik_panel, 'Position', [162 140 68 20], ...
    'Text', '0.060 m', 'HorizontalAlignment', 'right');

% θ4 wrist
uilabel(ik_panel, 'Position', [10 85 200 20], ...
    'Text', sprintf('θ₄ Wrist (°)  [%+d … %+d]', J4_MIN, J4_MAX));
sld_t4ik = uislider(ik_panel, 'Position', [10 73 210 3], ...
    'Limits', [J4_MIN J4_MAX], 'Value', 0, ...
    'ValueChangingFcn', @(~,e) updateIK([],[],[],e.Value));
lbl_t4ik = uilabel(ik_panel, 'Position', [162 85 68 20], ...
    'Text', '0°', 'HorizontalAlignment', 'right');

%% ─── Trajectory panel ───────────────────────────────────────────────
traj_panel = uipanel(fig, 'Title', 'Trajectory (quintic)', ...
    'Position', [10 10 250 275], 'FontWeight', 'bold');

uilabel(traj_panel, 'Position', [10 228 220 20], 'Text', 'Steps', 'FontSize', 11);
sld_steps = uislider(traj_panel, 'Position', [10 217 210 3], ...
    'Limits', [20 200], 'Value', 80, ...
    'ValueChangingFcn', @(~,e) setSteps(e.Value));
lbl_steps = uilabel(traj_panel, 'Position', [175 228 55 20], ...
    'Text', '80', 'HorizontalAlignment', 'right');

btn_traj = uibutton(traj_panel, 'push', ...
    'Position', [10 160 110 28], 'Text', 'Play', ...
    'ButtonPushedFcn', @(~,~) runTrajectory());
btn_reset = uibutton(traj_panel, 'push', ...
    'Position', [130 160 110 28], 'Text', 'Reset', ...
    'ButtonPushedFcn', @(~,~) resetPlot());

uilabel(traj_panel, 'Position', [10 135 230 20], ...
    'Text', 'End-Effector (DH FK):', 'FontWeight', 'bold');
lbl_ee = uilabel(traj_panel, 'Position', [10 65 230 70], ...
    'Text', 'X: —   Y: —   Z: —', 'FontSize', 10, 'WordWrap', 'on');

lbl_warn = uilabel(traj_panel, 'Position', [10 10 230 52], ...
    'Text', '', 'FontColor', [0.8 0.1 0.1], 'FontSize', 9, 'WordWrap', 'on');

%% ─── Shared state ────────────────────────────────────────────────────
state = struct();
state.mode    = 'fk';
% FK joint-space values (degrees / metres, joint frame)
state.t1      = 0;       % J1 joint deg  (home = arm along +Y)
state.t3      = 0;     % J3 joint deg
state.t4      = 0;       % J4 joint deg
state.d2      = 0;    % J2 metres
% IK target values
state.x_ik    = 0.25;
state.y_ik    = 0.25;
state.z_ik    = 0.06;
state.t4ik    = 0;
state.steps   = 80;
state.trail_x = [];
state.trail_y = [];
state.trail_z = [];

%% Initial draw
drawRobot(state);

%% ═══════════════════════════════════════════════════════════════════
%  CALLBACKS
%% ═══════════════════════════════════════════════════════════════════

    function updateFK(t1, t3, t4, d2)
        if ~isempty(t1), state.t1 = t1; lbl_t1.Text = sprintf('%+.0f°', t1); end
        if ~isempty(t3), state.t3 = t3; lbl_t3.Text = sprintf('%+.0f°', t3); end
        if ~isempty(t4), state.t4 = t4; lbl_t4.Text = sprintf('%+.0f°', t4); end
        if ~isempty(d2), state.d2 = d2; lbl_d2.Text = sprintf('%.3f m', d2); end
        state.trail_x = []; state.trail_y = []; state.trail_z = [];
        drawRobot(state);
    end

    function updateIK(x, y, z, t4)
        if ~isempty(x),  state.x_ik = x;  lbl_x.Text    = sprintf('%.3f m', x);  end
        if ~isempty(y),  state.y_ik = y;  lbl_y.Text    = sprintf('%.3f m', y);  end
        if ~isempty(z),  state.z_ik = z;  lbl_z.Text    = sprintf('%.3f m', z);  end
        if ~isempty(t4), state.t4ik = t4; lbl_t4ik.Text = sprintf('%+.0f°', t4); end
        state.trail_x = []; state.trail_y = []; state.trail_z = [];
        drawRobot(state);
    end

    function setSteps(v)
        state.steps = round(v);
        lbl_steps.Text = sprintf('%d', state.steps);
    end

    function toggleMode(btn)
        if btn.Value
            state.mode = 'ik';
            btn.Text = 'IK Mode  |  Switch to FK';
            fk_panel.Visible = 'off';
            ik_panel.Visible = 'on';
            title(ax, 'SCARA RPRR — IK Mode');
        else
            state.mode = 'fk';
            btn.Text = 'FK Mode  |  Switch to IK';
            fk_panel.Visible = 'on';
            ik_panel.Visible = 'off';
            title(ax, 'SCARA RPRR — FK Mode');
        end
        state.trail_x = []; state.trail_y = []; state.trail_z = [];
        drawRobot(state);
    end

    function resetPlot()
        state.trail_x = []; state.trail_y = []; state.trail_z = [];
        lbl_warn.Text = '';
        drawRobot(state);
    end

    function runTrajectory()
        % Clamp helper: keep joint values inside hardware limits
        clampJ = @(v, lo, hi) max(min(v, hi), lo);

        if strcmp(state.mode, 'fk')
            q_start = [state.t1, state.d2, state.t3, state.t4];
            % Demo end-pose: perturb within joint limits
            q_end = [clampJ(state.t1 + 40, J1_MIN, J1_MAX), ...
                     clampJ(state.d2 + 0.05, J2_MIN, J2_MAX), ...
                     clampJ(state.t3 - 30, J3_MIN, J3_MAX), ...
                     clampJ(state.t4 + 20, J4_MIN, J4_MAX)];
        else
            sol1 = ikSolveJoint(state.x_ik, state.y_ik, L1, L2, HOME_OFFSET);
            sol2 = ikSolveJoint(state.x_ik - 0.10, state.y_ik + 0.10, L1, L2, HOME_OFFSET);
            if isempty(sol1) || isempty(sol2)
                lbl_warn.Text = 'IK: one or both poses unreachable within workspace.';
                return;
            end
            % Check J3 stays within ±80°
            if abs(sol1(2)) > J3_MAX || abs(sol2(2)) > J3_MAX
                lbl_warn.Text = 'IK: elbow angle exceeds ±80° joint limit.';
                return;
            end
            q_start = [sol1(1), state.z_ik,              sol1(2), state.t4ik];
            q_end   = [sol2(1), clampJ(state.z_ik+0.04, J2_MIN, J2_MAX), ...
                       sol2(2), clampJ(state.t4ik+15, J4_MIN, J4_MAX)];
        end

        N = state.steps;
        Q = jtraj_simple(q_start, q_end, N);

        state.trail_x = zeros(1, N);
        state.trail_y = zeros(1, N);
        state.trail_z = zeros(1, N);

        for i = 1:N
            t1 = Q(i,1); d2 = Q(i,2); t3 = Q(i,3); t4 = Q(i,4);

            [ex, ey, ez] = fkFull(t1, d2, t3, t4, d1, HOME_OFFSET);
            state.trail_x(i) = ex;
            state.trail_y(i) = ey;
            state.trail_z(i) = ez;

            state.t1 = t1; state.t3 = t3; state.t4 = t4; state.d2 = d2;

            if strcmp(state.mode, 'fk')
                sld_t1.Value = t1; lbl_t1.Text = sprintf('%+.0f°', t1);
                sld_t3.Value = t3; lbl_t3.Text = sprintf('%+.0f°', t3);
                sld_t4.Value = t4; lbl_t4.Text = sprintf('%+.0f°', t4);
                sld_d2.Value = d2; lbl_d2.Text = sprintf('%.3f m', d2);
            end

            tmp = state;
            tmp.trail_x = state.trail_x(1:i);
            tmp.trail_y = state.trail_y(1:i);
            tmp.trail_z = state.trail_z(1:i);
            drawRobot(tmp);
            pause(0.01);
        end
    end

%% ═══════════════════════════════════════════════════════════════════
%  DRAWING
%% ═══════════════════════════════════════════════════════════════════

    function drawRobot(s)
        cla(ax);
        lbl_warn.Text = '';

        if strcmp(s.mode, 'fk')
            j1 = s.t1; d2 = s.d2; j3 = s.t3; j4 = s.t4;
        else
            sol = ikSolveJoint(s.x_ik, s.y_ik, L1, L2, HOME_OFFSET);
            if isempty(sol) || abs(sol(2)) > J3_MAX
                if isempty(sol)
                    lbl_warn.Text = 'Target outside reach (r_min/r_max).';
                else
                    lbl_warn.Text = sprintf('Elbow angle %.0f° exceeds ±80° limit!', sol(2));
                end
                j1=0; j3=0; j4=s.t4ik; d2=s.z_ik;
            else
                j1=sol(1); j3=sol(2); j4=s.t4ik; d2=s.z_ik;
            end
        end

        % ── Full DH forward kinematics ─────────────────────────────
        [ex, ey, ez, yaw_deg, j2x, j2y] = fkFullViz(j1, d2, j3, j4, d1, HOME_OFFSET, L1, L2);
        z_plane = d1 + d2;   % shoulder / arm Z height

        % ── Ground plane ──────────────────────────────────────────
        fill3(ax, [-0.5 0.5 0.5 -0.5], [-0.5 -0.5 0.5 0.5], [0 0 0 0], ...
              [0.93 0.93 0.93], 'EdgeColor', 'none', 'FaceAlpha', 0.45);

        % ── Base column (prismatic J2) ─────────────────────────────
        plot3(ax, [0 0], [0 0], [0 d1], ...
              'Color', [0.5 0.5 0.5], 'LineWidth', 4);
        % Prismatic extension indicator
        if d2 > 0.002
            plot3(ax, [0 0], [0 0], [d1 z_plane], ...
                  'Color', [0.7 0.7 0.7], 'LineWidth', 2.5, 'LineStyle', '--');
        end
        % Shoulder joint sphere
        plot3(ax, 0, 0, z_plane, 'o', 'MarkerSize', 16, ...
              'MarkerFaceColor', [0.4 0.6 0.9], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);

        % Home direction indicator (+Y)
        plot3(ax, [0, 0], [0, 0.12], [z_plane z_plane], ...
              'g--', 'LineWidth', 1.2);
        text(ax, 0.01, 0.13, z_plane, '+Y home', 'FontSize', 7, 'Color', [0 0.5 0]);

        % ── Link 1 ────────────────────────────────────────────────
        plot3(ax, [0 j2x], [0 j2y], [z_plane z_plane], ...
              'Color', [0.14 0.39 0.71], 'LineWidth', 8);

        % ── Link 2 ────────────────────────────────────────────────
        plot3(ax, [j2x ex], [j2y ey], [z_plane z_plane], ...
              'Color', [0.71 0.38 0.13], 'LineWidth', 8);

        % ── EE orientation indicator ──────────────────────────────
        ee_reach = 0.055;
        plot3(ax, [ex, ex + ee_reach*cosd(yaw_deg)], ...
                  [ey, ey + ee_reach*sind(yaw_deg)], ...
                  [ez, ez], 'r-', 'LineWidth', 2.0);

        % ── Joints ────────────────────────────────────────────────
        scatter3(ax,  0,   0,   z_plane, 130, [0.20 0.70 0.45], 'filled', 'MarkerEdgeColor', 'w');
        scatter3(ax, j2x, j2y, z_plane, 130, [0.20 0.70 0.45], 'filled', 'MarkerEdgeColor', 'w');
        scatter3(ax, ex,  ey,  ez,      100, [0.80 0.20 0.20],  'filled', 'MarkerEdgeColor', 'w');

        % ── EE-to-Z drop line (visual helper) ─────────────────────
        plot3(ax, [ex ex], [ey ey], [0 ez], ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);

        % ── Trail ─────────────────────────────────────────────────
        if ~isempty(s.trail_x)
            plot3(ax, s.trail_x, s.trail_y, s.trail_z, ...
                  '-', 'LineWidth', 1.8, 'Color', [0.8 0.2 0.2 0.55]);
            scatter3(ax, s.trail_x(1),   s.trail_y(1),   s.trail_z(1),   45, [0.14 0.39 0.71], 'filled');
            scatter3(ax, s.trail_x(end), s.trail_y(end), s.trail_z(end), 45, [0.8 0.2 0.2],    'filled');
        end

        % ── Labels ────────────────────────────────────────────────
        text(ax,  0.015,  0.015, z_plane+0.015, 'J1', 'FontSize', 9, 'Color', [0.2 0.2 0.2]);
        text(ax, j2x+0.015, j2y+0.015, z_plane+0.015, 'J3', 'FontSize', 9, 'Color', [0.2 0.2 0.2]);
        text(ax, ex+0.015,  ey+0.015,  ez+0.015, 'EE', 'FontSize', 9, 'Color', [0.7 0.1 0.1]);

        % ── EE info readout ───────────────────────────────────────
        lbl_ee.Text = sprintf( ...
            'X: %+.4f m\nY: %+.4f m\nZ: %+.4f m\nYaw: %+.1f°\nJ1: %+.1f°  J3: %+.1f°  J4: %+.1f°\nd₂: %.3f m', ...
            ex, ey, ez, yaw_deg, j1, j3, j4, d2);

        drawnow limitrate;
    end

%% ═══════════════════════════════════════════════════════════════════
%  KINEMATICS  (matching teleop.py compute_fk exactly)
%% ═══════════════════════════════════════════════════════════════════

    % Full DH FK — returns EE position + yaw (matches teleop compute_fk)
    % j1_joint_deg : J1 in JOINT space degrees (before home offset)
    % d2_m         : J2 prismatic in metres
    % j3_deg       : J3 in joint degrees
    % j4_deg       : J4 in joint degrees
    function [ex, ey, ez, yaw_deg] = fkFull(j1_joint_deg, d2_m, j3_deg, j4_deg, base_d, home_off)
        th1 = deg2rad(j1_joint_deg) + deg2rad(home_off);  % world frame
        th3 = deg2rad(j3_deg);
        th4 = deg2rad(j4_deg);
        d2  = d2_m;

        % DH table: [a, alpha, d, theta]
        dh = [0,     0,    base_d,   th1; ...   % J1
              0,     0,    d2,       0;   ...    % J2 prismatic
              0.24,  pi,   0,        th3; ...    % J3
              0.147, 0,   -0.06375,  th4; ...    % J4
              0,     0,   -0.1535,   0  ];       % EE offset

        T = eye(4);
        for row = 1:size(dh,1)
            a  = dh(row,1); al = dh(row,2);
            d  = dh(row,3); th = dh(row,4);
            ct = cos(th); st = sin(th);
            ca = cos(al); sa = sin(al);
            A = [ct, -st*ca,  st*sa, a*ct;
                 st,  ct*ca, -ct*sa, a*st;
                  0,  sa,     ca,    d;
                  0,  0,      0,     1];
            T = T * A;
        end
        ex      = T(1,4);
        ey      = T(2,4);
        ez      = T(3,4);
        yaw_deg = rad2deg(atan2(T(2,1), T(1,1)));
    end

    % Same as fkFull but also returns J2 elbow XY for drawing
    function [ex, ey, ez, yaw_deg, j2x, j2y] = fkFullViz(j1_joint_deg, d2_m, j3_deg, j4_deg, base_d, home_off, l1, ~)
        [ex, ey, ez, yaw_deg] = fkFull(j1_joint_deg, d2_m, j3_deg, j4_deg, base_d, home_off);
        % Elbow joint XY (end of Link 1, in world frame)
        th1_w = deg2rad(j1_joint_deg) + deg2rad(home_off);
        j2x   = l1 * cos(th1_w);
        j2y   = l1 * sin(th1_w);
    end

    % IK solver — operates in WORLD frame, returns [J1_joint_deg, J3_joint_deg]
    % IK is purely planar (SCARA); J2/J4 set directly by sliders.
    function sol = ikSolveJoint(x_world, y_world, l1, l2, home_off)
        c3 = (x_world^2 + y_world^2 - l1^2 - l2^2) / (2*l1*l2);
        if abs(c3) > 1
            sol = [];
            return;
        end
        j3_world = acosd(c3);                         % elbow-up, world frame
        j1_world = atan2d(y_world, x_world) ...
                   - atan2d(l2*sind(j3_world), l1 + l2*cosd(j3_world));
        j1_joint = j1_world - home_off;               % convert to joint space
        % Normalise J1 joint angle to [-180, 180]
        j1_joint = mod(j1_joint + 180, 360) - 180;
        sol = [j1_joint, j3_world];                   % [J1 joint deg, J3 world=joint deg]
    end

    function Q = jtraj_simple(q0, qf, n)
        t = linspace(0, 1, n)';
        s = 6*t.^5 - 15*t.^4 + 10*t.^3;   % quintic smoothstep
        Q = q0 + s .* (qf - q0);
    end

end  % simulation