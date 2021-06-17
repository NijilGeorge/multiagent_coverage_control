binaryImage = true(size(grayImage));
labeledImage = bwlabel(binaryImage);
measurements = regionprops(labeledImage, grayImage, 'WeightedCentroid')
centerOfMass = measurements.WeightedCentroid
hold on;
plot(centerOfMass(1), centerOfMass(2), 'r*', 'LineWidth', 2, 'MarkerSize', 16);