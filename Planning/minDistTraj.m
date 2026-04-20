function [alphaOpt] = minDistTraj(ref, n, twL, twR)
%MINDISTTRAJ Compute min. distance trajectory based on Race Driver Model
%paper (F. Braghin), using modified to use reference trajectory and
%trackwidths
% reference: trajectory to use as reference
% twL: left trackwidths
% twR: right trackwidths
% n: normals from reference
% alphaOpt: computed alpha for constructing the trajectory as 
%           trackLimRight + alpha*(trackLimLeft-trackLimRight)

N = size(ref,1);

H = zeros(N,N);
B = zeros(1,N);

for i=1:N-1
    dNix = [n(i+1,1);-n(i,1)];
    dNiy = [n(i+1,2);-n(i,2)];
    dXi = ref(i+1,1)-ref(i,1);
    dYi = ref(i+1,2)-ref(i,2);

    Hsi=zeros(N,N);
    Bsi=zeros(1,N);

    Hsi(i:i+1,i:i+1)=dNix*dNix.'+dNiy*dNiy.';
    Bsi(1,i:i+1)=2*dXi*dNix.'+2*dYi*dNiy.';

    H = H + 2*Hsi; % The paper uses a'Ha+B'a, we want 1/2a'Ha+B'a
    B = B + Bsi;
end


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
%       s.t. l <= Ax <= u
prob = osqp;
prob.setup(P, q, A, l, u);

% Solve
res = prob.solve();
alphaOpt = res.x;

end