within DistrictHeating.Pipes;
model AvgTempPlugFlowPipe
  "Calculates average temperature inside a pipe with a certain plug flow profile"
  extends IDEAS.Fluid.Interfaces.PartialTwoPortInterface;

  parameter Modelica.SIunits.Length pipeLength;
  parameter Modelica.SIunits.Length pipeDiameter;
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal;

  parameter Modelica.SIunits.Density rho = 1000;
  final parameter Modelica.SIunits.Area pipeArea = Modelica.Constants.pi*pipeDiameter^2/4
    "Cross section of pipe";

  parameter Modelica.SIunits.Temperature TInit = 273+70
    "Initial temperature of the water in the pipe";

  PlugFlowLosslessPipe plug(
    L=pipeLength,
    D=pipeDiameter,
    m_flow_nominal=m_flow_nominal,
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Annex60.Fluid.Sensors.MassFlowRate senMasFlo(
      redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));

  Annex60.Fluid.Sensors.TemperatureTwoPort senTemIn(m_flow_nominal=m_flow_nominal,
      redeclare package Medium = Medium,
    T_start=TInit)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Annex60.Fluid.Sensors.TemperatureTwoPort senTemOut(m_flow_nominal=m_flow_nominal,
      redeclare package Medium = Medium,
    T_start=TInit)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Blocks.Continuous.Integrator integrator(k=1, initType=Modelica.Blocks.Types.Init.InitialOutput)
    annotation (Placement(transformation(extent={{28,60},{48,80}})));
  Modelica.Blocks.Math.Product product
    annotation (Placement(transformation(extent={{-6,60},{14,80}})));
  Modelica.Blocks.Sources.RealExpression NormalizedVelocity(y=senMasFlo.m_flow/(
        rho*pipeArea)/pipeLength) "Calculates normalized velocity v/L"
    annotation (Placement(transformation(extent={{-80,66},{-60,86}})));
  Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-30,50})));
  Modelica.Blocks.Interfaces.RealOutput TAvg "Average temperature in degC"
    annotation (Placement(transformation(extent={{96,46},{116,66}})));
  Modelica.Blocks.Math.Add add1       annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={74,56})));
  Modelica.Blocks.Sources.RealExpression InitialT(y=TInit - 273.15)
    "Initial temperature in pipe"
    annotation (Placement(transformation(extent={{24,32},{44,52}})));
equation
  connect(port_a, senMasFlo.port_a) annotation (Line(
      points={{-100,0},{-80,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTemIn.port_b, plug.port_a) annotation (Line(
      points={{-20,0},{0,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senMasFlo.port_b, senTemIn.port_a) annotation (Line(
      points={{-60,0},{-40,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug.port_b, senTemOut.port_a) annotation (Line(
      points={{20,0},{40,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTemOut.port_b, port_b) annotation (Line(
      points={{60,0},{100,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(NormalizedVelocity.y, product.u1) annotation (Line(
      points={{-59,76},{-8,76}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(product.y, integrator.u) annotation (Line(
      points={{15,70},{26,70}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(add.y, product.u2) annotation (Line(
      points={{-30,61},{-30,64},{-8,64}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(senTemIn.T, add.u1) annotation (Line(
      points={{-30,11},{-30,24},{-36,24},{-36,38}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(senTemOut.T, add.u2) annotation (Line(
      points={{50,11},{50,28},{-24,28},{-24,38}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(integrator.y, add1.u1) annotation (Line(
      points={{49,70},{54,70},{54,62},{62,62}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(add1.u2, InitialT.y) annotation (Line(
      points={{62,50},{54,50},{54,42},{45,42}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(add1.y, TAvg) annotation (Line(
      points={{85,56},{106,56}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
        Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={192,192,192}),
        Rectangle(
          extent={{-100,50},{100,-48}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={217,236,256}),
        Rectangle(
          extent={{-20,50},{20,-48}},
          lineColor={175,175,175},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175})}));
end AvgTempPlugFlowPipe;
