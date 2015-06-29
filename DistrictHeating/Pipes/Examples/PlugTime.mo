within DistrictHeating.Pipes.Examples;
model PlugTime

  Modelica.SIunits.Time tin;
  Modelica.SIunits.Time tout;
  Modelica.SIunits.Time td;

  Real x "Normalized transport quantity";

  Modelica.Blocks.Interfaces.RealInput u
    annotation (Placement(transformation(extent={{-130,-20},{-90,20}})));
equation
  //Speed
  der(x) = u;

  tin = time;
  td = time - tout;

  //Spatial distribution of the time
  (,tout) =
    spatialDistribution(
      time,
      0,
      x,
      true,
      {0.0, 0.5, 1},
      {0.0, -0.5, -1});

end PlugTime;
