function build_simulink_model()
% BUILD_SIMULINK_MODEL  Programmatically build the 3-DOF Simulink model.
%
% This script constructs a Simulink model that simulates the football-
% kicking robot's leg control loop with realistic servo dynamics:
%
%   target (x,z)  -->  3-DOF IK  -->  3x servo dynamics  -->  actual angles
%                                          (2nd-order TF)            |
%                                                         hip,knee  -->  FK  -->  actual (x,z)
%
% Servo dynamics model:
%   G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
%   wn   = 30 rad/s     natural frequency
%   zeta = 0.7          damping ratio
%   --> realistic SG90-like step response (~0.2 s settling, ~5% overshoot)
%
% The model has 5 scopes:
%   1. Hip_Angle    : target vs actual hip angle (deg)
%   2. Knee_Angle   : target vs actual knee angle (deg)
%   3. Ankle_Angle  : target vs actual ankle angle (deg)
%   4. Foot_X       : target vs actual foot x position (cm)
%   5. Foot_Z       : target vs actual foot z position (cm)
%
% USAGE:
%   >> cd simulink_sim
%   >> build_simulink_model        % creates kick_servo_dynamics_3dof.slx
%   >> open_system('kick_servo_dynamics_3dof')
%   >> sim('kick_servo_dynamics_3dof')
%   % Then double-click each Scope block to see results.

    mdl = 'kick_servo_dynamics_3dof';

    % --- 1. Clean up any existing version -------------------------------
    try, close_system(mdl, 0); catch, end                     %#ok<NOCOM>
    if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end

    % --- 2. Create new model and configure simulation -------------------
    new_system(mdl);
    set_param(mdl, 'StopTime', '5.0', 'Solver', 'ode45');

    % Pre-load kick waypoints into base workspace when the simulation starts.
    % Format for From Workspace block: [time, value; time, value; ...]
    %
    % Kick sequence:
    %   t=0.0..0.5 s   neutral pose      (0, 11.9)
    %   t=1.5 s        wind-up           (-2, 8)
    %   t=2.0 s        impact at ball    (3, 9)
    %   t=2.5 s        follow-through    (5, 7)
    %   t=5.0 s        back to neutral   (0, 11.9)
    init_code = [ ...
        'x_target = [0 0; 0.5 0; 1.5 -2; 2.0 3; 2.5 5; 5.0 0];' newline ...
        'z_target = [0 11.9; 0.5 11.9; 1.5 8; 2.0 9; 2.5 7; 5.0 11.9];'];
    set_param(mdl, 'InitFcn', init_code);

    % --- 3. Add source blocks (kick waypoints) --------------------------
    add_block('simulink/Sources/From Workspace', [mdl '/x_target'], ...
        'VariableName', 'x_target', ...
        'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [30 30 100 60]);

    add_block('simulink/Sources/From Workspace', [mdl '/z_target'], ...
        'VariableName', 'z_target', ...
        'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [30 100 100 130]);

    % --- 4. 3-DOF IK MATLAB Function block ------------------------------
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [mdl '/IK_3DOF'], ...
        'Position', [180 30 290 130]);

    ik_code = [ ...
        'function [hip, knee, ankle] = fcn(x, z)'                                  newline ...
        '%% 3-DOF Inverse Kinematics (matches firmware kinematics.cpp)'            newline ...
        'L1 = 7.0;  L2 = 5.0;'                                                     newline ...
        'z2 = hypot(x, z);'                                                        newline ...
        'hip = 0; knee = 0; ankle = 90;'                                           newline ...
        'if z2 >= L1 + L2 || z2 <= abs(L1 - L2)'                                   newline ...
        '    return;  % unreachable -> hold previous safe defaults'                newline ...
        'end'                                                                      newline ...
        'ha = atan2(x, z);'                                                        newline ...
        'cos_hb = (L1^2 + z2^2 - L2^2) / (2*L1*z2);'                              newline ...
        'cos_hb = max(min(cos_hb, 1), -1);'                                        newline ...
        'hb = acos(cos_hb);'                                                       newline ...
        'hip = rad2deg(ha + hb);'                                                  newline ...
        'cos_k = (L1^2 + L2^2 - z2^2) / (2*L1*L2);'                               newline ...
        'cos_k = max(min(cos_k, 1), -1);'                                          newline ...
        'knee = rad2deg(pi - acos(cos_k));'                                        newline ...
        'cos_a = (L2^2 + z2^2 - L1^2) / (2*L2*z2);'                               newline ...
        'cos_a = max(min(cos_a, 1), -1);'                                          newline ...
        'ankle = rad2deg(pi/2 + ha - acos(cos_a));'                                newline ...
        'end'];
    set_matlab_function_body([mdl '/IK_3DOF'], ik_code);

    % --- 5. Three servo dynamics blocks (2nd-order transfer functions) --
    % G(s) = 900 / (s^2 + 42 s + 900)   -- wn=30, zeta=0.7
    servo_num = '[900]';
    servo_den = '[1 42 900]';

    add_block('simulink/Continuous/Transfer Fcn', [mdl '/Servo_Hip'], ...
        'Numerator', servo_num, 'Denominator', servo_den, ...
        'Position', [360 25 450 65]);

    add_block('simulink/Continuous/Transfer Fcn', [mdl '/Servo_Knee'], ...
        'Numerator', servo_num, 'Denominator', servo_den, ...
        'Position', [360 95 450 135]);

    add_block('simulink/Continuous/Transfer Fcn', [mdl '/Servo_Ankle'], ...
        'Numerator', servo_num, 'Denominator', servo_den, ...
        'Position', [360 165 450 205]);

    % --- 6. Forward kinematics block (hip + knee -> foot position) ------
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [mdl '/FK_2DOF'], ...
        'Position', [600 50 710 130]);

    fk_code = [ ...
        'function [foot_x, foot_z] = fcn(hip, knee)'                               newline ...
        '%% Forward Kinematics: actual hip+knee -> actual foot position'           newline ...
        'L1 = 7.0;  L2 = 5.0;'                                                     newline ...
        'h = deg2rad(hip);'                                                        newline ...
        'k = deg2rad(knee);'                                                       newline ...
        'shin = h - k;'                                                            newline ...
        'foot_x = L1*sin(h) + L2*sin(shin);'                                       newline ...
        'foot_z = L1*cos(h) + L2*cos(shin);'                                       newline ...
        'end'];
    set_matlab_function_body([mdl '/FK_2DOF'], fk_code);

    % --- 7. Mux blocks (combine target + actual for each scope) ---------
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Hip'], ...
        'Inputs', '2', 'Position', [490 25 495 65]);

    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Knee'], ...
        'Inputs', '2', 'Position', [490 95 495 135]);

    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Ankle'], ...
        'Inputs', '2', 'Position', [490 165 495 205]);

    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_FootX'], ...
        'Inputs', '2', 'Position', [770 35 775 75]);

    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_FootZ'], ...
        'Inputs', '2', 'Position', [770 105 775 145]);

    % --- 8. Scope blocks ------------------------------------------------
    add_block('simulink/Sinks/Scope', [mdl '/Hip_Angle'],   'Position', [540 25 580 65]);
    add_block('simulink/Sinks/Scope', [mdl '/Knee_Angle'],  'Position', [540 95 580 135]);
    add_block('simulink/Sinks/Scope', [mdl '/Ankle_Angle'], 'Position', [540 165 580 205]);
    add_block('simulink/Sinks/Scope', [mdl '/Foot_X'],      'Position', [820 35 860 75]);
    add_block('simulink/Sinks/Scope', [mdl '/Foot_Z'],      'Position', [820 105 860 145]);

    % --- 9. Connect everything with auto-routing ------------------------
    o = {'autorouting','on'};

    % Sources -> IK
    add_line(mdl, 'x_target/1', 'IK_3DOF/1', o{:});
    add_line(mdl, 'z_target/1', 'IK_3DOF/2', o{:});

    % IK outputs -> Servo TFs
    add_line(mdl, 'IK_3DOF/1', 'Servo_Hip/1',   o{:});
    add_line(mdl, 'IK_3DOF/2', 'Servo_Knee/1',  o{:});
    add_line(mdl, 'IK_3DOF/3', 'Servo_Ankle/1', o{:});

    % IK outputs -> Mux (target angle traces)
    add_line(mdl, 'IK_3DOF/1', 'Mux_Hip/1',   o{:});
    add_line(mdl, 'IK_3DOF/2', 'Mux_Knee/1',  o{:});
    add_line(mdl, 'IK_3DOF/3', 'Mux_Ankle/1', o{:});

    % Servo outputs -> Mux (actual angle traces)
    add_line(mdl, 'Servo_Hip/1',   'Mux_Hip/2',   o{:});
    add_line(mdl, 'Servo_Knee/1',  'Mux_Knee/2',  o{:});
    add_line(mdl, 'Servo_Ankle/1', 'Mux_Ankle/2', o{:});

    % Servo hip+knee -> FK
    add_line(mdl, 'Servo_Hip/1',  'FK_2DOF/1', o{:});
    add_line(mdl, 'Servo_Knee/1', 'FK_2DOF/2', o{:});

    % Mux -> angle scopes
    add_line(mdl, 'Mux_Hip/1',   'Hip_Angle/1',   o{:});
    add_line(mdl, 'Mux_Knee/1',  'Knee_Angle/1',  o{:});
    add_line(mdl, 'Mux_Ankle/1', 'Ankle_Angle/1', o{:});

    % Foot target + actual -> Mux -> scopes
    add_line(mdl, 'x_target/1',  'Mux_FootX/1', o{:});
    add_line(mdl, 'FK_2DOF/1',   'Mux_FootX/2', o{:});
    add_line(mdl, 'z_target/1',  'Mux_FootZ/1', o{:});
    add_line(mdl, 'FK_2DOF/2',   'Mux_FootZ/2', o{:});

    add_line(mdl, 'Mux_FootX/1', 'Foot_X/1', o{:});
    add_line(mdl, 'Mux_FootZ/1', 'Foot_Z/1', o{:});

    % --- 10. Save -------------------------------------------------------
    save_system(mdl, [mdl '.slx']);

    fprintf('\n');
    fprintf('=================================================================\n');
    fprintf(' Simulink model built successfully: %s.slx\n', mdl);
    fprintf('=================================================================\n');
    fprintf(' Next steps:\n');
    fprintf('   1.  open_system(''%s'')\n', mdl);
    fprintf('   2.  Press Run (or:  sim(''%s'')  )\n', mdl);
    fprintf('   3.  Double-click each Scope block to view results.\n');
    fprintf('\n');
    fprintf(' Each angle scope shows TWO traces:\n');
    fprintf('   yellow = target angle from IK (step changes)\n');
    fprintf('   blue   = actual servo angle (smooth 2nd-order response)\n');
    fprintf('=================================================================\n');
end


% ====================================================================
% Helper: set the function body inside a MATLAB Function block
% ====================================================================
function set_matlab_function_body(block_path, code)
    sf_root = sfroot;
    chart = sf_root.find('-isa','Stateflow.EMChart','Path',block_path);
    if isempty(chart)
        error('Could not find MATLAB Function chart at: %s', block_path);
    end
    chart.Script = code;
end
