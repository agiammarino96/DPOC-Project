function stateIndex = ComputePickUpStateIndex(stateSpace, map)
%ComputePickUpStateIndex Compute the index of the pickup state in the
%stateSpace matrix
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       stateIndex:
%           An integer that is the index of the pickup state in the
%           stateSpace matrix

global PICK_UP
     [M,N]=size(map); % size of the world
     
     % check for each element (i,j) of the world in order to find the
     % one correspondent to "pickup" cell.
  
     for i=1:M
         for j=1:N
             if map(i,j)==PICK_UP
                 m=i;
                 n=j;
                 break
             end
         end
     end
     
     % find the index of the row of state space matrix correspondent
     % to the state where cell is "pickup" and the quadricopter is
     % not carrying the package (psi=0)
     stateIndex = find(ismember(stateSpace, [m n 0],'rows'));
end