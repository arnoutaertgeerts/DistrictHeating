within DistrictHeating.Pipes.Examples;
model PlugTimeTest
  PlugTime plugTime
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Sources.Pulse pulse(period=100, offset=0.1)
    annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
  Modelica.Blocks.Continuous.Filter filter(f_cut=0.01)
    annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
equation
  connect(plugTime.u, filter.y) annotation (Line(
      points={{-11,0},{-31,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(filter.u, pulse.y) annotation (Line(
      points={{-54,0},{-65,0}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end PlugTimeTest;
