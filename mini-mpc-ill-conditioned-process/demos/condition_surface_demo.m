% condition_surface_demo.m
% Demo: Visualize the condition number surface for a 2x2 parameterized matrix
% Shows how small changes in gain matrix elements can cause large changes
% in condition number -- the hallmark of ill-conditioned MPC problems.

% Parameterized 2x2 gain matrix: G = [a, b; c, d]
% Vary b and d around nominal values to see condition surface

a = 1.0; c = 1.0;  % Fixed parameters
b_vals = linspace(-0.99, -1.01, 100);
d_vals = linspace(-0.99, -1.01, 100);
[B, D] = meshgrid(b_vals, d_vals);

Kappa = zeros(size(B));
for i = 1:length(b_vals)
    for j = 1:length(d_vals)
        G = [a, B(i,j); c, D(i,j)];
        Kappa(i,j) = cond(G);
    end
end

figure;
surf(B, D, log10(Kappa));
xlabel('G(1,2) = b'); ylabel('G(2,2) = d');
zlabel('log_{10}(kappa)');
title('Condition Number Surface for 2x2 Gain Matrix');
colorbar;

% Mark the singular point where det(G) = 0 (a*d - b*c = 0 => d = b*c/a)
hold on;
b_line = b_vals;
d_line = b_line * c / a;
plot3(b_line, d_line, log10(max(Kappa(:)))*ones(size(b_line)), 'r-', 'LineWidth', 2);
legend('kappa surface', 'singularity (det=0)');

saveas(gcf, 'condition_surface.png');
disp('Demo complete: condition surface saved as condition_surface.png');
