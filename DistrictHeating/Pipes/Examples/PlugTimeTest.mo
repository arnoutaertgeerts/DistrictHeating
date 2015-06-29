within DistrictHeating.Pipes.Examples;
model PlugTimeTest
  PlugTime plugTime
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Sources.Step step(
    startTime=10,
    offset=1,
    height=-0.5)
    annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
equation
  connect(plugTime.u, step.y) annotation (Line(
      points={{-11,0},{-27,0}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end PlugTimeTest;
