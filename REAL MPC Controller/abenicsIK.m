function theta_ref = abenicsIK(q_des, params)
%ABENICSIK Convert desired CS gear orientation into four output-side MP angles.
%
% Input:
%   q_des  = [roll; pitch; yaw] in radians
%   params = ABENICS parameter structure
%
% Output:
%   theta_ref = [theta_rA; theta_pA; theta_rB; theta_pB] in radians
%
% Project convention:
%   theta_ref is output-side MP-gear angle reference, not raw motor angle.
%
% Motor order is always:
%   theta_ref = [theta_rA;
%                theta_pA;
%                theta_rB;
%                theta_pB];

    % -----------------------------
    % Input checks
    % -----------------------------


    %ryan - just checking the size and shi
    if ~isequal(size(q_des), [3, 1])
        error('abenicsIK:q_desSize', ...
              'q_des must be a 3x1 vector: [roll; pitch; yaw].');
    end

    if ~isfield(params, 'beta')
        error('abenicsIK:MissingBeta', ...
              ['params.beta is required. ', ...
               'beta is the ABENICS paper angle formed by the two driving modules.']);
    end


    %ryan - converts the inputs into non vector varaibles
    % -----------------------------
    % Read desired CS-gear orientation
    % Paper convention:
    %   BRH = Rx(r) * Ry(p) * Rz(y)
    % -----------------------------
    r = q_des(1);   % roll about world X, radians
    p = q_des(2);   % pitch about world Y, radians
    y = q_des(3);   % yaw about world Z, radians

    beta = params.beta;   % paper beta: angle formed by the two driving modules %ryan -  stolen from the params, lwk idk why not just defined where but balright

    %ryan - random trig bs
    % -----------------------------
    % Trig shorthand
    % Paper notation:
    %   Cr = cos(r), Sr = sin(r), etc.
    % -----------------------------
    Cr = cos(r);
    Sr = sin(r);

    Cp = cos(p);
    Sp = sin(p);

    Cy = cos(y);
    Sy = sin(y);

    Cbeta = cos(beta);
    Sbeta = sin(beta);

    %%ryan -  IMPORTANT IMPORTANT IMPORTANT WTF IS THIS
    % Small tolerance for singular atan2(0,0) cases
    tol = 1e-12;

    % ok figured it out, basically matlab is stupid so can produce perfect
    % numbers, so instead we just say is the error below tolerance, and if
    % it is, we're good


    %ryan - after this, the math outside my area of expertise


    % ============================================================
    % Module A inverse kinematics
    % Paper Eq. 54:
    % theta_A1 = atan((Cr*Sy + Cy*Sp*Sr) / (Cr*Cy*Sp - Sr*Sy))
    %
    % MATLAB implementation:
    % use atan2(numerator, denominator)
    % ============================================================
    A1_num = Cr*Sy + Cy*Sp*Sr;
    A1_den = Cr*Cy*Sp - Sr*Sy;
    theta_A1 = localAtan2Safe(A1_num, A1_den, tol);

    % ============================================================
    % Paper Eq. 55:
    % theta_A2 = acos(Cp*Cy)
    % ============================================================
    A2_arg = localClamp(Cp*Cy, -1, 1);
    theta_A2 = acos(A2_arg);

    % ============================================================
    % Paper Eq. 56:
    % theta_A3 = -atan((Cp*Sy) / Sp)
    %
    % Not used in theta_ref because theta_A3 is not one of the
    % four active output-side MP gear angles.
    % Kept here as documentation of the full paper IK chain.
    % ============================================================
    A3_num = Cp*Sy;
    A3_den = Sp;
    theta_A3 = -localAtan2Safe(A3_num, A3_den, tol); %#ok<NASGU>

    % ============================================================
    % Module B inverse kinematics
    % Paper Eq. 57:
    % theta_B1 = -atan( numerator / denominator )
    %
    % numerator:
    %   Cy*Cbeta*Cr + Sy*(Sbeta*Cp - Cbeta*Sp*Sr)
    %
    % denominator:
    %   Cr*Sp*Sy + Cy*Sr
    % ============================================================
    B1_num = Cy*Cbeta*Cr + Sy*(Sbeta*Cp - Cbeta*Sp*Sr);
    B1_den = Cr*Sp*Sy + Cy*Sr;
    theta_B1 = -localAtan2Safe(B1_num, B1_den, tol);

    % ============================================================
    % Paper Eq. 58:
    % theta_B2 = acos(Cy*Cr*Sbeta - Sy*(Cbeta*Cp + Sbeta*Sp*Sr))
    % ============================================================
    B2_arg = Cy*Cr*Sbeta - Sy*(Cbeta*Cp + Sbeta*Sp*Sr);
    B2_arg = localClamp(B2_arg, -1, 1);
    theta_B2 = acos(B2_arg);

    % ============================================================
    % Paper Eq. 59:
    % theta_B3 = atan( numerator / denominator )
    %
    % Not used in theta_ref because theta_B3 is not one of the
    % four active output-side MP gear angles.
    % Kept here as documentation of the full paper IK chain.
    % ============================================================
    B3_num = Cy*(Cbeta*Cp + Sbeta*Sp*Sr) + Sy*Sbeta*Cr;
    B3_den = -Cbeta*Sp + Sbeta*Cp*Sr;
    theta_B3 = localAtan2Safe(B3_num, B3_den, tol); %#ok<NASGU>

    % ============================================================
    % Paper Eq. 17-18 relationship to output-side MP gear angles
    %
    % theta_rA = theta_A1
    % theta_pA = -2*theta_A2
    % theta_rB = theta_B1
    % theta_pB = -2*theta_B2
    %
    % Output order must remain:
    % [theta_rA; theta_pA; theta_rB; theta_pB]
    % ============================================================
    theta_rA = theta_A1;
    theta_pA = -2*theta_A2;

    theta_rB = theta_B1;
    theta_pB = -2*theta_B2;

    theta_ref = [theta_rA;
                 theta_pA;
                 theta_rB;
                 theta_pB];

    % Final output size check
    if ~isequal(size(theta_ref), [4, 1])
        error('abenicsIK:thetaRefSize', ...
              'theta_ref must be a 4x1 vector.');
    end
end


%ryan - huh what this
% ------------------------------------------------------------
% Helper: clamp value into [lower, upper]
% Used before acos to prevent floating-point domain errors.
% ------------------------------------------------------------
function x_clamped = localClamp(x, lower, upper)
    x_clamped = min(upper, max(lower, x));
end

%this just makes sure that the cos is accepting a good range

% ------------------------------------------------------------
% Helper: safe atan2
%
% The paper writes atan(numerator/denominator).
% MATLAB uses atan2(numerator, denominator) for quadrant safety.
%
% If both numerator and denominator are near zero, the angle is
% not uniquely defined. For early simulation, return 0 as a
% branch convention.
% ------------------------------------------------------------

%ryan - what that is
function angle = localAtan2Safe(num, den, tol)
    if abs(num) < tol && abs(den) < tol
        angle = 0;
    else
        angle = atan2(num, den);
    end
end

