function [U, r, lrms] = MyKabsch(P, Q, m)
%
% INPUT
% P: n x 3 matrix of vertices
% Q: n x 3 matrix of vertices
% m: n x 1 row vector of weights
%
% OUTPUT
% U: Rotation matrix
% r: Translation vector
% lrms: Achieved RMS error
%
% When calling [U, r] = MyKabsch(P, Q), the vertices of P can be aligned
% with the vertices of Q by computing
% Q ~= P * U + repmat(r, [size(P, 1), 1]);
%

    if nargin < 3
        nPoints = size(P, 1);
		m = ones(1, nPoints) / nPoints ;
    end

    [U, r, lrms] = Kabsch(P', Q', m);
    U = U';
    r = r';
end
