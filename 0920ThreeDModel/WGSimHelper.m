testnumbers = 1:17;
% testForces = [repmat([50 0 0 0 0 0],4,1);
%               repmat([0 50 0 0 0 0],4,1);
%               repmat([0 0 50 0 0 0],4,1);
%               repmat([0 0 0 5 0 0],4,1);
%               repmat([0 0 0 0 5 0],4,1);
%               repmat([0 0 0 0 0 5],4,1)];
          
% testFocus = [repmat([1;2;3;4],6,1)];
winddirs = 190:10:350;
% sphases = 0:10:350;

for iter = 1:length(testnumbers)
    testnumber = testnumbers(iter);
%     testForce = [testForces(iter,:),testFocus(iter)];
    winddir = winddirs(iter);
%     sphase = sphases(iter);
    WGSim;
    clearvars -except testnumbers testForces testFocus iter winddirs sphases
end