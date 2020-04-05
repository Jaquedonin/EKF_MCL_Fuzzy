function [path] = geraTraj(filename)
    image = imread(filename);
    grayimage = rgb2gray(image);
    bwimage = grayimage < 0.5;
    grid = robotics.BinaryOccupancyGrid(bwimage, 100);

    % Infla o OcuppancyGrid map
    robotRadius = 0.40;
    mapInflated = copy(grid);
    inflate(mapInflated,robotRadius);

    prm = robotics.PRM;
    prm.Map = mapInflated;

    startLocation = [1.17 0];
    endLocation = [5.5 2.5];

    prm.NumNodes = 2500;
    prm.ConnectionDistance = 0.18;

    path = findpath(prm, startLocation, endLocation);
    
    while isempty(path)
        prm.NumNodes = prm.NumNodes + 10;
        update(prm);
        path = findpath(prm, startLocation, endLocation);
    end
end

