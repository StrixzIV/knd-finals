%% SCARA RPRR Interactive Simulator with Sliders
% Requires: Robotics Toolbox (Peter Corke) or pure FK below
% MATLAB R2016b+ for uifigure/uislider

function simulation()
clear; clc; close all;

%% Robot parameters
L1 = 0.4;   % Link 1 length (m)
L2 = 0.3;   % Link 2 length (m)
d1 = 0.1;   % Fixed base height (m)

%% Build the figure
fig = uifigure('Name', 'SCARA RPRR Simulator', ...
               'Position', [100 100 900 650]);

%% Axes for robot plot
ax = uiaxes(fig, 'Position', [260 20 620 610]);
ax.XLim = [-0.8 0.8]; ax.YLim = [-0.8 0.8]; ax.ZLim = [-0.1 0.6];
ax.XLabel.String = 'X (m)';
ax.YLabel.String = 'Y (m)';
ax.ZLabel.String = 'Z (m)';
grid(ax, 'on'); hold(ax, 'on'); view(ax, 45, 30);
title(ax, 'SCARA RPRR — FK Mode');

%% Mode toggle
mode_label = uilabel(fig, 'Position', [10 610 230 25], ...
    'Text', 'Mode', 'FontWeight', 'bold', 'FontSize', 13);

mode_btn = uibutton(fig, 'state', ...
    'Position', [10 580 230 28], ...
    'Text', 'FK Mode  |  Switch to IK', ...
    'FontSize', 11, ...
    'ValueChangedFcn', @(btn,evt) toggleMode(btn));

%% ── FK Sliders ──────────────────────────────────────────
fk_panel = uipanel(fig, 'Title', 'Forward Kinematics', ...
    'Position', [10 280 240 290], ...
    'FontWeight', 'bold');

% θ1
uilabel(fk_panel, 'Position', [10 240 160 20], 'Text', 'θ₁  Base (°)');
sld_t1 = uislider(fk_panel, 'Position', [10 230 200 3], ...
    'Limits', [-180 180], 'Value', 30, ...
    'ValueChangingFcn', @(sld,evt) updateFK(evt.Value, [], [], []));
lbl_t1 = uilabel(fk_panel, 'Position', [170 240 60 20], ...
    'Text', '30°', 'HorizontalAlignment', 'right');

% θ3
uilabel(fk_panel, 'Position', [10 185 160 20], 'Text', 'θ₃  Elbow (°)');
sld_t3 = uislider(fk_panel, 'Position', [10 175 200 3], ...
    'Limits', [-170 170], 'Value', -60, ...
    'ValueChangingFcn', @(sld,evt) updateFK([], evt.Value, [], []));
lbl_t3 = uilabel(fk_panel, 'Position', [170 185 60 20], ...
    'Text', '-60°', 'HorizontalAlignment', 'right');

% θ4
uilabel(fk_panel, 'Position', [10 130 160 20], 'Text', 'θ₄  Wrist (°)');
sld_t4 = uislider(fk_panel, 'Position', [10 120 200 3], ...
    'Limits', [-180 180], 'Value', 0, ...
    'ValueChangingFcn', @(sld,evt) updateFK([], [], evt.Value, []));
lbl_t4 = uilabel(fk_panel, 'Position', [170 130 60 20], ...
    'Text', '0°', 'HorizontalAlignment', 'right');

% d2
uilabel(fk_panel, 'Position', [10 75 160 20], 'Text', 'd₂  Height (m)');
sld_d2 = uislider(fk_panel, 'Position', [10 65 200 3], ...
    'Limits', [0 0.3], 'Value', 0.05, ...
    'ValueChangingFcn', @(sld,evt) updateFK([], [], [], evt.Value));
lbl_d2 = uilabel(fk_panel, 'Position', [160 75 70 20], ...
    'Text', '0.05 m', 'HorizontalAlignment', 'right');

%% ── IK Sliders ──────────────────────────────────────────
ik_panel = uipanel(fig, 'Title', 'Inverse Kinematics', ...
    'Position', [10 280 240 290], ...
    'FontWeight', 'bold', 'Visible', 'off');

% X target
uilabel(ik_panel, 'Position', [10 240 160 20], 'Text', 'X target (m)');
sld_x = uislider(ik_panel, 'Position', [10 230 200 3], ...
    'Limits', [-0.65 0.65], 'Value', 0.4, ...
    'ValueChangingFcn', @(sld,evt) updateIK(evt.Value, [], [], []));
lbl_x = uilabel(ik_panel, 'Position', [160 240 70 20], ...
    'Text', '0.40 m', 'HorizontalAlignment', 'right');

% Y target
uilabel(ik_panel, 'Position', [10 185 160 20], 'Text', 'Y target (m)');
sld_y = uislider(ik_panel, 'Position', [10 175 200 3], ...
    'Limits', [-0.65 0.65], 'Value', 0.2, ...
    'ValueChangingFcn', @(sld,evt) updateIK([], evt.Value, [], []));
lbl_y = uilabel(ik_panel, 'Position', [160 185 70 20], ...
    'Text', '0.20 m', 'HorizontalAlignment', 'right');

% Z (d2) target
uilabel(ik_panel, 'Position', [10 130 160 20], 'Text', 'd₂ Height (m)');
sld_z = uislider(ik_panel, 'Position', [10 120 200 3], ...
    'Limits', [0 0.3], 'Value', 0.05, ...
    'ValueChangingFcn', @(sld,evt) updateIK([], [], evt.Value, []));
lbl_z = uilabel(ik_panel, 'Position', [160 130 70 20], ...
    'Text', '0.05 m', 'HorizontalAlignment', 'right');

% θ4 wrist
uilabel(ik_panel, 'Position', [10 75 160 20], 'Text', 'θ₄  Wrist (°)');
sld_t4ik = uislider(ik_panel, 'Position', [10 65 200 3], ...
    'Limits', [-180 180], 'Value', 0, ...
    'ValueChangingFcn', @(sld,evt) updateIK([], [], [], evt.Value));
lbl_t4ik = uilabel(ik_panel, 'Position', [160 75 70 20], ...
    'Text', '0°', 'HorizontalAlignment', 'right');

%% ── Trajectory Panel ─────────────────────────────────────
traj_panel = uipanel(fig, 'Title', 'Trajectory (jtraj)', ...
    'Position', [10 10 240 260], 'FontWeight', 'bold');

uilabel(traj_panel, 'Position', [10 215 220 20], ...
    'Text', 'Steps', 'FontSize', 11);
sld_steps = uislider(traj_panel, 'Position', [10 205 200 3], ...
    'Limits', [20 200], 'Value', 80, ...
    'ValueChangingFcn', @(sld,evt) setSteps(evt.Value));
lbl_steps = uilabel(traj_panel, 'Position', [170 215 60 20], ...
    'Text', '80', 'HorizontalAlignment', 'right');

btn_traj = uibutton(traj_panel, 'push', ...
    'Position', [10 145 100 28], 'Text', 'Play', ...
    'ButtonPushedFcn', @(btn,evt) runTrajectory());

btn_reset = uibutton(traj_panel, 'push', ...
    'Position', [120 145 100 28], 'Text', 'Reset', ...
    'ButtonPushedFcn', @(btn,evt) resetPlot());

% EE info display
uilabel(traj_panel, 'Position', [10 125 220 20], ...
    'Text', 'End-Effector Position:', 'FontWeight', 'bold');
lbl_ee = uilabel(traj_panel, 'Position', [10 60 220 65], ...
    'Text', 'X: —   Y: —   Z: —', ...
    'FontSize', 11, 'WordWrap', 'on');

% Singularity warning
lbl_warn = uilabel(traj_panel, 'Position', [10 20 220 35], ...
    'Text', '', 'FontColor', [0.8 0.1 0.1], ...
    'FontSize', 10, 'WordWrap', 'on');

%% ── Shared state ─────────────────────────────────────────
state = struct();
state.mode    = 'fk';
state.t1      = 30;
state.t3      = -60;
state.t4      = 0;
state.d2      = 0.05;
state.x_ik    = 0.4;
state.y_ik    = 0.2;
state.z_ik    = 0.05;
state.t4ik    = 0;
state.steps   = 80;
state.trail_x = [];
state.trail_y = [];
state.trail_z = [];

%% Initial draw
drawRobot(state);

%% ═══════════════════════════════════════════════════════
%  CALLBACKS
%% ═══════════════════════════════════════════════════════

    function updateFK(t1, t3, t4, d2)
        if ~isempty(t1), state.t1 = t1; lbl_t1.Text = sprintf('%.0f°', t1); end
        if ~isempty(t3), state.t3 = t3; lbl_t3.Text = sprintf('%.0f°', t3); end
        if ~isempty(t4), state.t4 = t4; lbl_t4.Text = sprintf('%.0f°', t4); end
        if ~isempty(d2), state.d2 = d2; lbl_d2.Text = sprintf('%.3f m', d2); end
        state.trail_x = []; state.trail_y = []; state.trail_z = [];
        drawRobot(state);
    end

    function updateIK(x, y, z, t4)
        if ~isempty(x),  state.x_ik = x;  lbl_x.Text    = sprintf('%.3f m', x);  end
        if ~isempty(y),  state.y_ik = y;  lbl_y.Text    = sprintf('%.3f m', y);  end
        if ~isempty(z),  state.z_ik = z;  lbl_z.Text    = sprintf('%.3f m', z);  end
        if ~isempty(t4), state.t4ik = t4; lbl_t4ik.Text = sprintf('%.0f°', t4); end
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
        % Build start and end joint configs from current slider state
        if strcmp(state.mode, 'fk')
            q_start = [state.t1, state.d2, state.t3, state.t4];
            % End: perturb angles for demo if no second pose defined
            q_end   = [state.t1+60, state.d2+0.1, state.t3-40, state.t4+30];
            q_end   = max(min(q_end, [180, 0.3, 170, 180]), [-180, 0, -170, -180]);
        else
            % IK mode: solve start and a shifted target
            sol1 = ikSolve2D(state.x_ik, state.y_ik, L1, L2);
            sol2 = ikSolve2D(state.x_ik - 0.15, state.y_ik + 0.15, L1, L2);
            if isempty(sol1) || isempty(sol2)
                lbl_warn.Text = 'IK: one or both poses unreachable.';
                return;
            end
            q_start = [sol1(1), state.z_ik,     sol1(2), state.t4ik];
            q_end   = [sol2(1), state.z_ik+0.08, sol2(2), state.t4ik+20];
        end

        N = state.steps;
        Q = jtraj_simple(q_start, q_end, N);   % quintic interpolation

        state.trail_x = zeros(1,N);
        state.trail_y = zeros(1,N);
        state.trail_z = zeros(1,N);

        for i = 1:N
            t1 = Q(i,1); d2 = Q(i,2); t3 = Q(i,3); t4 = Q(i,4);
            [ex, ey] = fkXY(t1, t3, L1, L2);
            state.trail_x(i) = ex;
            state.trail_y(i) = ey;
            state.trail_z(i) = d1 + d2;

            % Update state for drawing
            state.t1 = t1; state.t3 = t3; state.t4 = t4; state.d2 = d2;

            % Sync sliders if in FK mode
            if strcmp(state.mode, 'fk')
                sld_t1.Value = t1; lbl_t1.Text = sprintf('%.0f°', t1);
                sld_t3.Value = t3; lbl_t3.Text = sprintf('%.0f°', t3);
                sld_d2.Value = d2; lbl_d2.Text = sprintf('%.3f m', d2);
            end

            % Trim trail to current frame
            tmp = state;
            tmp.trail_x = state.trail_x(1:i);
            tmp.trail_y = state.trail_y(1:i);
            tmp.trail_z = state.trail_z(1:i);
            drawRobot(tmp);
            pause(0.01);
        end
    end

%% ═══════════════════════════════════════════════════════
%  DRAWING
%% ═══════════════════════════════════════════════════════

    function drawRobot(s)
        cla(ax);
        lbl_warn.Text = '';

        if strcmp(s.mode, 'fk')
            t1 = s.t1; t3 = s.t3; t4 = s.t4; d2 = s.d2;
        else
            sol = ikSolve2D(s.x_ik, s.y_ik, L1, L2);
            if isempty(sol)
                lbl_warn.Text = 'Target unreachable — outside workspace!';
                t1=0; t3=0; t4=s.t4ik; d2=s.z_ik;
            else
                t1=sol(1); t3=sol(2); t4=s.t4ik; d2=s.z_ik;
            end
        end

        [ex, ey] = fkXY(t1, t3, L1, L2);
        j2x = L1*cosd(t1);
        j2y = L1*sind(t1);
        z   = d1 + d2;

        % Ground plane
        fill3(ax, [-0.7 0.7 0.7 -0.7], [-0.7 -0.7 0.7 0.7], [0 0 0 0], ...
              [0.94 0.94 0.94], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

        % Vertical column (prismatic d2)
        plot3(ax, [0 0], [0 0], [0 d1], 'Color', [0.5 0.5 0.5], ...
              'LineWidth', 3);
        plot3(ax, 0, 0, z, 'o', 'MarkerSize', 14, ...
              'MarkerFaceColor', [0.4 0.6 0.9], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);

        % Link 1
        plot3(ax, [0 j2x], [0 j2y], [z z], 'Color', [0.14 0.39 0.71], 'LineWidth', 7);

        % Link 2
        plot3(ax, [j2x ex], [j2y ey], [z z], 'Color', [0.71 0.38 0.13], 'LineWidth', 7);

        % EE orientation line
        ee_angle = t1 + t3 + t4;
        plot3(ax, [ex ex+0.06*cosd(ee_angle)], ...
                  [ey ey+0.06*sind(ee_angle)], [z z], ...
              'r--', 'LineWidth', 1.5);

        % Joints
        scatter3(ax, 0,   0,   z, 120, [0.2 0.7 0.45], 'filled', 'MarkerEdgeColor', 'w');
        scatter3(ax, j2x, j2y, z, 120, [0.2 0.7 0.45], 'filled', 'MarkerEdgeColor', 'w');
        scatter3(ax, ex,  ey,  z,  90, [0.8 0.2 0.2],  'filled', 'MarkerEdgeColor', 'w');

        % Trail
        if ~isempty(s.trail_x)
            plot3(ax, s.trail_x, s.trail_y, s.trail_z, ...
                  'r-', 'LineWidth', 1.5, 'Color', [0.8 0.2 0.2 0.5]);
            scatter3(ax, s.trail_x(1), s.trail_y(1), s.trail_z(1), ...
                     40, [0.14 0.39 0.71], 'filled');
            scatter3(ax, s.trail_x(end), s.trail_y(end), s.trail_z(end), ...
                     40, [0.8 0.2 0.2], 'filled');
        end

        % Labels
        text(ax, 0+0.02,   0+0.02,   z+0.02, 'J1', 'FontSize', 10, 'Color', [0.3 0.3 0.3]);
        text(ax, j2x+0.02, j2y+0.02, z+0.02, 'J2', 'FontSize', 10, 'Color', [0.3 0.3 0.3]);
        text(ax, ex+0.02,  ey+0.02,  z+0.02, 'EE', 'FontSize', 10, 'Color', [0.7 0.1 0.1]);

        % EE readout
        lbl_ee.Text = sprintf('X: %.3f m\nY: %.3f m\nZ: %.3f m', ex, ey, z);

        drawnow limitrate;
    end

%% ═══════════════════════════════════════════════════════
%  KINEMATICS HELPERS  (no toolbox needed)
%% ═══════════════════════════════════════════════════════

    function [ex, ey] = fkXY(t1_deg, t3_deg, l1, l2)
        ex = l1*cosd(t1_deg) + l2*cosd(t1_deg + t3_deg);
        ey = l1*sind(t1_deg) + l2*sind(t1_deg + t3_deg);
    end

    function sol = ikSolve2D(x, y, l1, l2)
        c3 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2);
        if abs(c3) > 1, sol = []; return; end
        t3 = acosd(c3);   % elbow-up solution
        t1 = atan2d(y, x) - atan2d(l2*sind(t3), l1 + l2*cosd(t3));
        sol = [t1, t3];
    end

    function Q = jtraj_simple(q0, qf, n)
        % Quintic polynomial trajectory (same as MATLAB's jtraj)
        t  = linspace(0, 1, n)';
        s  = 6*t.^5 - 15*t.^4 + 10*t.^3;   % smoothstep
        Q  = q0 + s .* (qf - q0);
    end

end  % main function