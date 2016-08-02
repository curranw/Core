data = csvread('data_visualization.csv');
map = imread('maze_map.pgm');

[x,y] = find(map ~= 205);
map_smallish = map(min(x):max(x), min(y):max(y));
[j, k] = size(map_smallish);
offset_x = 0.4; 
offset_y = -0.8;
[Xs, Ys] = meshgrid(linspace( -k/2*.05, k/2*.05, k)+offset_x, linspace(j/2*.05, -j/2*.05, j)+offset_y);

figure(10000)
clf
%h1 = pcolor(Xs,Ys, map_smallish);
%hold on
%shading interp
%colormap(gray)

x = data(:,1);
y = data(:,2);
c = data(:,5);
n = (c - min(c))/(max(c) - min(c)) * 100;

%h2 = scatter(x,y,200,n,'fill');
% colormap(autumn)
%hold on;

l_theta = length(unique(data(:,3)));
l_x = length(unique(data(:,1)));
l_y = length(unique(data(:,2)));

map_c = zeros(l_x, l_y);
for i = 0:l_x-1
    for j = 0:l_y-1
        x_start = l_y* l_theta * i + j * l_theta;
        map_c(i+1,j+1) = sum(data(x_start+1:x_start+l_theta,5));
    end
end


imagesc(unique(x), unique(y), map_c')
set(gca, 'YDir', 'normal')
hold on;
for i = 0:2
    ind = find(data(:,4) == i);
    x = data(ind, 1);
    y = data(ind, 2);
    [u,v] = pol2cart(data(ind,3), 1);
    if i == 0
        quiver(x,y,u,v, 'color',[0 0 1]);
    end
    if i == 1
       quiver(x,y,u,v, 'color',[0 1 0]); 
    end
    if i == 2
       quiver(x,y,u,v, 'color',[1 0 0]); 
    end
    hold on;
end

legend('Straight', 'Rotate Right', 'Rotate Left');