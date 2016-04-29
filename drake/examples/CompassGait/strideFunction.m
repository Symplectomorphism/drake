function xf = strideFunction(p, x0)
traj = simulate(p,[0 1], [1; x0]); % Simulate from x0 with mode = 1
xf = traj.traj{2}.eval(traj.traj{2}.tspan(1)); % Get state right after collision
end