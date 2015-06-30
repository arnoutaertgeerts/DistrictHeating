within DistrictHeating.Pipes.Examples;
model PlugTime

  Modelica.SIunits.Time tin;
  Modelica.SIunits.Time tout;
  Modelica.SIunits.Time td;

  Real x "Normalized transport quantity";
  Real tau "Time delay";

  Modelica.Blocks.Interfaces.RealInput u
    annotation (Placement(transformation(extent={{-130,-20},{-90,20}})));

  Modelica.Blocks.Nonlinear.VariableDelay delay
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=tau)
    annotation (Placement(transformation(extent={{-40,-16},{-20,4}})));

initial equation
  tau = 1/u;

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

  der(tau) = 1 - u/delay.y;

  connect(u, delay.u) annotation (Line(
      points={{-110,0},{-12,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(delay.delayTime, realExpression.y) annotation (Line(
      points={{-12,-6},{-19,-6}},
      color={0,0,127},
      smooth=Smooth.None));
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end PlugTime;
