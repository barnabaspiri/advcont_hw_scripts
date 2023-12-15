function [A, B] = TASK0_ss_AB(M, C, K, F)
%% 1st order DDE generation from system matrices
% M: mass matrix (nxn)
% C: damping matrix (nxn)
% K: stiffness matrix (nxn)
% F: excitation vector (1xn)
dof = length(F);
    if dof == 1
        A = zeros(2, 2);
        A(1,2) = 1;
        A(2,1) = -(K+F)/M;
        A(2,2) = -C/M;
        B(1) = 0;
        B(2) = F/M;
    else
        A = zeros(dof*2,dof*2);
        A(1:dof,(dof+1):2*dof) = eye(dof);
        A((dof+1):2*dof,1:dof) = -inv(M)*K;
        A((dof+1):2*dof,(dof+1):2*dof) = -inv(M)*C;
        A([2,3],:) = A([3,2],:);
        A(:,[2,3]) = A(:,[3,2]);
        B(1:2*dof,1) = zeros(2*dof,1);
        B(dof+1:2*dof,1) = inv(M)*F;
        temp = B(2);
        B(2) = B(3);
        B(3) = temp;
    end
end