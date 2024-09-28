function plotGradient(t, yArray)
% input an index t, and an array to plot line by line
% https://www.mathworks.com/matlabcentral/answers/1440139-how-to-set-plot-color-for-n-curves-to-be-a-gradient-of-n-color-shades-between-light-blue-and-dark-bl
lightBLUE = [0.356862745098039,0.811764705882353,0.956862745098039];
darkBLUE = [0.0196078431372549,0.0745098039215686,0.670588235294118];
red = [1 0 0];
purpel = [0.4 0 .6];
medium_green = [0.47, 0.67, 0.19];
dark_blue = [0, 0.2, 0.6];

% test=blueGRADIENTflexible((1:N),N);
N = size(yArray,1);

% if N < 10
%     GRADIENTflexible = @(n,nn) interp1([1/nn 1],[medium_green;purpel],n/nn);
% else
    GRADIENTflexible = @(n,nn) interp1([1/nn 1],[lightBLUE;darkBLUE],n/nn);
% end

for i = 1:N
    plot(t,yArray(i,:),'color',GRADIENTflexible(i,N))
    hold on
end

yValues = min(min(yArray)):.1:max(max(yArray));
plot(zeros(length(yValues),1), yValues, "--")
plot(zeros(length(yValues),1)+2, yValues, "--")