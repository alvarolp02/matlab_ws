function [alphaOpt] = minCurvTraj(ref, n, twL, twR)
%MINCURVTRAJ Compute min. curvature trajectory based on Race Driver Model
%paper (F. Braghin), using modified to use reference trajectory and
%trackwidths
% reference: trajectory to use as reference
% twL: left trackwidths
% twR: right trackwidths
% n: normals from reference
% alphaOpt: computed alpha for constructing the trajectory as 
%           trackLimRight + alpha*(trackLimLeft-trackLimRight)

N = size(ref,1);

Nx = diag(n(:,1));
Ny = diag(n(:,2));

D = -2.*eye(N,N) + diag(ones(1,N-1),1) + diag(ones(1,N-1),-1);
D(1,end)=1;
D(end,1)=1;

xref = ref(:,1);
yref = ref(:,2);

H = (Nx.'*(D.'*D)*Nx + Ny.'*(D.'*D)*Ny);

B = 2.*(xref.'*(D.'*D)*Nx + yref.'*(D.'*D)*Ny);

% Convert to sparse (OSQP prefers sparse matrices)
P = sparse(H); % H must be symmetric (pos. semidefinite)
q = B;

% -twR <= alpha <= twL
% Constraints in form l <= Ax <= u
A = speye(N);  % sparse identity
l = -ones(N,1).*twR;
u = ones(N,1).*twL;

% Setup solver
% Solve min in x : (1/2)x'Px + q'x 
prob = osqp;
prob.setup(P, q, A, l, u);

% Solve
res = prob.solve();
alphaOpt = res.x;

end