function [alphaOpt] = minDistTraj(trackLimLeft,trackLimRight)
%MINDISTTRAJ Compute min. distance trajectory based on Race Driver Model
%paper (F. Braghin)
% trackLimLeft: left limit for trajectory
% trackLimRight: right limit for trajectory
% alphaOpt: computed alpha for constructing the trajectory as 
%           trackLimRight + alpha*(trackLimLeft-trackLimRight)

deltaX = trackLimLeft(:,1)-trackLimRight(:,1);
deltaY = trackLimLeft(:,2)-trackLimRight(:,2);

N = size(trackLimLeft,1);

H = zeros(N,N);
B = zeros(1,N);

for i=1:N-1
    deltaXi = [deltaX(i+1);-deltaX(i)];
    deltaYi = [deltaY(i+1);-deltaY(i)];
    deltaXRi = trackLimRight(i+1,1)-trackLimRight(i,1);
    deltaYRi = trackLimRight(i+1,2)-trackLimRight(i,2);

    Hsi=zeros(N,N);
    Bsi=zeros(1,N);

    Hsi(i:i+1,i:i+1)=deltaXi*deltaXi.'+deltaYi*deltaYi.';
    Bsi(1,i:i+1)=2*deltaXRi*deltaXi.'+2*deltaYRi*deltaYi.';

    H = H + 2*Hsi; % The paper uses a'Ha+B'a, we want 1/2a'Ha+B'a
    B = B + Bsi;
end


% Convert to sparse (OSQP prefers sparse matrices)
P = sparse(H); % H must be symmetric (pos. semidefinite)
q = B;

% 0 <= alpha <= 1
% Constraints in form l <= Ax <= u
A = speye(N);  % sparse identity
l = zeros(N,1);
u = ones(N,1);

% Setup solver
% Solve min in x : (1/2)x'Px + q'x 
%       s.t. l <= Ax <= u
prob = osqp;
prob.setup(P, q, A, l, u);

% Solve
res = prob.solve();
alphaOpt = res.x;

end