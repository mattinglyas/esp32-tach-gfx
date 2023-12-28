% simple test of projection used for tachometer animation

theta = pi / 2; % degree FOV
screenx = 480;
screeny = 320;
a = screeny / screenx; % h/w aspect ratio

% all z coordinates must fall within here
znear = 3;
zfar = 50;

% some parameters
lm = (zfar / (zfar - znear));
f = 1 / tan(theta / 2);

% transformation matrix
t = [a*f 0 0 0; 0 f 0 0; 0 0 lm -lm*znear; 0 0 1 0];

% some test coordinates
x = linspace(-14, 14, 17);
z = linspace(4, 12, 9);
[X, Z] = meshgrid(x, z);
Y = ones(size(X)) * -2;
W = ones(size(X));
coords = [X(:) Y(:) Z(:) W(:)]';

% project those fellas
proj = t * coords;

% perspect those fellas
ncol = size(proj, 2);
persp = proj;
for k = 1:ncol
    persp(:,k) = proj(:, k) ./ proj(4, k);
end

% convert to pixels
pxels = persp;
pxels(1, :) = pxels(1, :) .* screenx + screenx / 2;
pxels(2, :) = pxels(2, :) .* screeny + screeny / 2;

% to draw
todraw = round([pxels(1, :); pxels(2, :)]);
screen = [0, 0, screenx, screenx; 0, screeny, 0, screeny];

close all
hold on
figure(1)
scatter(todraw(1, :), todraw(2, :));
scatter(screen(1, :), screen(2, :));

figure(2)
scatter(todraw(1, :), todraw(2, :));
xlim([0, screenx]);
ylim([0, screeny]);

% actual results
todraw2 = [todraw(1, :); screeny - todraw(2, :)];