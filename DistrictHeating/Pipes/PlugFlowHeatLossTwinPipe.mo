within DistrictHeating.Pipes;
model PlugFlowHeatLossTwinPipe
  "Pipe model with a temperature plug flow, pressure losses and heat exchange to the environment"

  //Extensions
  extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations(
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState);

  extends IDEAS.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1=Medium,
    redeclare final package Medium2=Medium,
    m1_flow_nominal=m_flow_nominal,
    m2_flow_nominal=m_flow_nominal);

  //Parameters
  parameter Modelica.SIunits.Length L;
  parameter Modelica.SIunits.Length D;
  parameter Real ha;
  parameter Real hs;

  final constant Real pi = Modelica.Constants.pi;
  final parameter Modelica.SIunits.Volume V=L*pi*(D/2)^2;

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal;
  parameter Modelica.SIunits.PressureDifference dp_nominal=0;

  parameter Modelica.SIunits.Density rho = 1000 "Mass density of fluid";
  parameter Modelica.SIunits.SpecificHeatCapacity cp=4187
    "Specific heat of fluid";

  parameter Boolean dynamicBalance = true
    "Set to true to use a dynamic balance, which often leads to smaller systems of equations"
    annotation (Evaluate=true, Dialog(tab="Dynamics", group="Equations"));
  parameter Modelica.SIunits.ThermalConductivity k=0.026 "Heat conductivity";

  final parameter Modelica.SIunits.ThermalConductivity Ra=1/(k*2*pi*ha);
  final parameter Modelica.SIunits.ThermalConductivity Rs=1/(k*2*pi*hs);

  final parameter Real C=rho*pi*(D/2)^2*cp;

  //Variables
  Real u;
  Modelica.SIunits.Power Q_Losses;
  Modelica.SIunits.Power Q_1;
  Modelica.SIunits.Power Q_2;

  //Components
  DistrictHeating.Pipes.PlugFlowPipe pipe1(
    pipeLength=L,
    pipeDiameter=D,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));

  Modelica.Blocks.Interfaces.RealInput TBoundary annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,110}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,100})));
  TimeDelays.PDETime pDETime
    annotation (Placement(transformation(extent={{-50,8},{-30,28}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=u)
    annotation (Placement(transformation(extent={{-80,8},{-60,28}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort senTem1(
    m_flow_nominal=m_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{-24,70},{-4,50}})));
  Annex60.Fluid.MixingVolumes.MixingVolume    idealHeater1(
    m_flow_nominal=m_flow_nominal,
    redeclare package Medium = Medium,
    nPorts=2,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    V=V)
    annotation (Placement(transformation(extent={{60,60},{80,40}})));
  DistrictHeating.Pipes.PlugFlowPipe pipe2(
    pipeLength=L,
    pipeDiameter=D,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{80,-70},{60,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort senTem2(
                                                m_flow_nominal=m_flow_nominal,
      redeclare package Medium = Medium,
    tau=0)
    annotation (Placement(transformation(extent={{10,-70},{-10,-50}})));
  Annex60.Fluid.MixingVolumes.MixingVolume       idealHeater2(
    m_flow_nominal=m_flow_nominal,
    redeclare package Medium = Medium,
    nPorts=2,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    V=V)
    annotation (Placement(transformation(extent={{-60,-60},{-80,-40}})));
  BaseClasses.TwinExponentialDecay twinExponentialDecay(
    C=C,
    Ra=Ra,
    Rs=Rs)
    annotation (Placement(transformation(extent={{12,0},{32,20}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
    annotation (Placement(transformation(extent={{-44,-56},{-56,-44}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature1
    annotation (Placement(transformation(extent={{44,44},{56,56}})));
equation
  //Normalized speed of the fluid [1/s]
  u = port_a1.m_flow/(rho*V);
  Q_Losses = -idealHeater1.heatPort.Q_flow/L -idealHeater2.heatPort.Q_flow/L;
  Q_1 = -idealHeater1.heatPort.Q_flow/L;
  Q_2 = -idealHeater2.heatPort.Q_flow/L;

  connect(pDETime.u, realExpression.y) annotation (Line(
      points={{-52,18},{-59,18}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pipe1.port_b, senTem1.port_a) annotation (Line(
      points={{-40,60},{-24,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe1.port_a, port_a1) annotation (Line(
      points={{-60,60},{-100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe2.port_b, senTem2.port_a) annotation (Line(
      points={{60,-60},{10,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe2.port_a, port_a2) annotation (Line(
      points={{80,-60},{100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pDETime.tau, twinExponentialDecay.td) annotation (Line(
      points={{-29,18},{10,18}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TBoundary, twinExponentialDecay.Tb) annotation (Line(
      points={{0,110},{0,14},{10,14}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(senTem1.T, twinExponentialDecay.T1) annotation (Line(
      points={{-14,49},{-14,8},{10,8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(senTem2.T, twinExponentialDecay.T2) annotation (Line(
      points={{0,-49},{0,4},{10,4}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(senTem1.port_b, idealHeater1.ports[1])
    annotation (Line(points={{-4,60},{68,60}}, color={0,127,255}));
  connect(idealHeater1.ports[2], port_b1)
    annotation (Line(points={{72,60},{72,60},{100,60}}, color={0,127,255}));
  connect(senTem2.port_b, idealHeater2.ports[1])
    annotation (Line(points={{-10,-60},{-68,-60}}, color={0,127,255}));
  connect(idealHeater2.ports[2], port_b2) annotation (Line(points={{-72,-60},{-72,
          -60},{-100,-60}}, color={0,127,255}));
  connect(idealHeater2.heatPort, prescribedTemperature.port)
    annotation (Line(points={{-60,-50},{-56,-50}}, color={191,0,0}));
  connect(idealHeater1.heatPort, prescribedTemperature1.port)
    annotation (Line(points={{60,50},{56,50}}, color={191,0,0}));
  connect(twinExponentialDecay.T1Out, prescribedTemperature1.T) annotation (
      Line(points={{33,14},{40,14},{40,50},{42.8,50}}, color={0,0,127}));
  connect(prescribedTemperature.T, twinExponentialDecay.T2Out) annotation (Line(
        points={{-42.8,-50},{-34,-50},{-34,-40},{40,-40},{40,6},{33,6}}, color={
          0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}),
                   graphics={
        Rectangle(
          extent={{-100,90},{100,28}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={192,192,192}),
        Rectangle(
          extent={{-100,84},{100,34}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255}),
        Rectangle(
          extent={{-26,84},{28,34}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Line(
          points={{55,-85},{-60,-85}},
          color={0,128,255},
          smooth=Smooth.None,
          visible=showDesignFlowDirection),
        Rectangle(
          extent={{-100,100},{100,90}},
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-100,28},{100,18}},
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-100,-28},{100,-90}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={192,192,192}),
        Rectangle(
          extent={{-100,-34},{100,-84}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255}),
        Rectangle(
          extent={{-100,-18},{100,-28}},
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-100,-90},{100,-100}},
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-26,-34},{28,-84}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Polygon(
          points={{8,70},{8,50},{28,60},{8,70}},
          lineColor={175,175,175},
          smooth=Smooth.None,
          fillColor={0,128,255},
          fillPattern=FillPattern.Forward),
        Line(
          points={{8,60},{-90,60}},
          color={175,175,175},
          smooth=Smooth.None),
        Polygon(
          points={{-6,-50},{-6,-70},{-26,-60},{-6,-50}},
          lineColor={175,175,175},
          smooth=Smooth.None,
          fillColor={0,128,255},
          fillPattern=FillPattern.Forward),
        Line(
          points={{92,-60},{-6,-60}},
          color={175,175,175},
          smooth=Smooth.None)}),
                           Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end PlugFlowHeatLossTwinPipe;
