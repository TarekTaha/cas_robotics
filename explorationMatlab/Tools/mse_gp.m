%% mse_gp
%
% *Description*: This file does basically the same as mse except is only
% for telling the mse of a matrix of doubles

%% Function Call
%
% *Inputs:* vals (size?, type?) Either a vector or matrix
%
% *Outputs:* mse_val (size?, type?) :the output mse value
%

function [mse_val]=mse_gp(vals)

%% Variables

%% Input Checks
if nargin<1
  error('need to pass in correct num of vars')
end

%% Functions
% Get the sum of all the values squared
numerator = sum(sum(vals.^2));
%denominator
numElements =numel(vals);
%The mean * 
mse_val = numerator / numElements;
