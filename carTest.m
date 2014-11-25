% Create a car with 2 trailers
C = CarWithNTrailers(2, [1, 2, 4]);

% Plot two states of the car
C.draw([1;0;pi/8;-pi/4;-pi/8; 0.5]);
hold on;
C.draw([10;3;-pi/8;pi/6;0; -0.3]);