classdef ControlLaw < TimedVector

   % Class to describe a zero order hold control law
   
   properties (SetAccess = private)
      Car;
   end
   
   methods
      function obj = ControlLaw(Car, ts, u)
        obj = obj@TimedVector(Car.InputDimension, ts, u);
        obj.Car = Car;
      end
   end 
   
end