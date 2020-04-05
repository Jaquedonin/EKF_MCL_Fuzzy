function [path] = geraTraj(filename)
    image = imread('mapa.bmp');
    grayimage = rgb2gray(image);
    bwimage = grayimage < 0.5;
    grid = robotics.BinaryOccupancyGrid(bwimage, 100);
    show(grid);
    
    % Infla o OcuppancyGrid map
    robotRadius = 0.45;
    mapInflated = copy(grid);
    inflate(mapInflated,robotRadius);
    show(mapInflated);
    
    prm = robotics.PRM;
    prm.Map = mapInflated;

    startLocation = [1.22 0];
    endLocation = [5.5 2.5];

    prm.NumNodes = 3500;
    prm.ConnectionDistance = 0.2;
    show(prm);

    path = findpath(prm, startLocation, endLocation);
    path;
    
    while isempty(path)
        prm.NumNodes = prm.NumNodes + 10;
        update(prm);
        path = findpath(prm, startLocation, endLocation);
    end


    path;

    show(prm);
    prm.NumNodes;
end
