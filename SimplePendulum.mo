within ;
model SimplePendulum " Model of a simple pendulum"

  constant Modelica.SIunits.Acceleration g = 9.81 "Gravitational constant";
  parameter Modelica.SIunits.Length L = 1 "Lenght of the pendulum";

  // Start of the variables

  Modelica.SIunits.Angle Theta(start=0.1, fixed=true);
  Modelica.SIunits.AngularVelocity ThetaDot;

equation

  ThetaDot = der(Theta);
  der(ThetaDot) = -g/L * sin(Theta);

end SimplePendulum;
