function [delta, S, w_n] = controller_constraints(Ts, overshoot_max)
    delta = .7;
    S = 100 * exp ((-pi*delta)/sqrt(1-delta^2));
    
    % Check the respect of the overshoot constraint
    if S >= overshoot_max
        error('Error! The controller design does not satisfy overshoot constraint.');
    end
    
    w_n = 3/(delta*Ts);     % natural frequency
end