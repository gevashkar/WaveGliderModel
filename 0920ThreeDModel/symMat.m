function M = symMat(dims,Mname,type)
% Builds an n-dimensional matrix of symbolic elements.
% M = symMat(dims,Mname)
% M = symMat(dims,Mname,type)
%
% dims is a vector containing the dimensionality of the matrix, [d1 d2 d3
% ...]. Mname is a character string that names the elements, i.e., 'm' or
% 'element'. type can be any valid input into the sym command.
%
% Returned in M is a matrix consisting of indexed symbolic elements, each
% element is named as Mname followed by its index location in the matrix.
%
% EX
%
% M = symMat([3 2],'m','real')
% M =
% [ m11, m12]
% [ m21, m22]
% [ m31, m32]
%
% class(M)
% ans =
% sym
%
% v = symMat(3,'v')
% v =
%  v1
%  v2
%  v3
% 
% v = symMat([1 3],'v')
% v =
% [ v11, v21, v31]
%
% See also
% sym syms class
%
%
% %%% ZCD Feb 2010 %%%
% edits: fixed index printing reversal for matrices with dimensions
%   greater than 9 - ZCD Nov 2013
%

% obtain a list of all possible indicies for a max(dim)#max(dim) matrix
elm = nchoosekr(1:max(dims),length(dims));
% maintain dimensional consistency for vectors
if length(dims)==1, elm = elm'; end
elmp = [];
for i =1:size(elm,1)
   elmp = [elmp; perms(elm(i,:))];  
end

% update total number of elements
elms = unique(elmp,'rows');
% place them into cells
elms = mat2cell(elms,ones(size(elms,1),1),length(dims));
% if matrix indicies have double digits, we need a spacer
if any(dims>9), spacer='_'; else spacer=''; end
% convert element indicies to strings
elms = cellfun(@(u) [Mname strrep(num2str(fliplr(u)),' ',spacer)],elms,'UniformOutput',false);
% reshape into desired dimensionality
if length(dims)~=1
    elms = reshape( elms',repmat(max(dims),[1 length(dims)]) );
end
% discard unwanted elements
bds = sprintf('1:%d,',dims);
eval(['elms = elms(' bds(1:end-1) ');']);

% create symbolic matrix
if nargin == 3
    M = sym(elms,type);
else
    M = sym(elms);
end



function y = nchoosekr(v,n)
   % y = nchoosekr(v,n)
   % The combinations of v things taken n at a time with replacement.

   if n == 1
      y = v;
   else
      v = v(:);
      y = [];
      m = length(v);
      if m == 1
         y = zeros(1, n);
         y(:) = v;
      else
         for i = 1 : m
         y_recr = nchoosekr(v(i:end), n-1);
         s_repl = zeros(size(y_recr, 1), 1);
         s_repl(:) = v(i);
         y = [ y ; s_repl, y_recr ];
         end
      end
   end


