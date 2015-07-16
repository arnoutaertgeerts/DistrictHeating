within DistrictHeating.Pipes;
model PlugFlowHeatLosses
  "Pipe model with a temperature plug flow, pressure losses and heat exchange to the environment"

  //Extensions
  extends IDEAS.Fluid.Interfaces.PartialTwoPortInterface;
  extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations(
    final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState);
  extends IDEAS.Fluid.Interfaces.TwoPortFlowResistanceParameters;

  //Parameters
  parameter Modelica.SIunits.Length L;
  parameter Modelica.SIunits.Length D;
  parameter Modelica.SIunits.Length h=0.02 "Insulation thickness";

  final constant Real pi = Modelica.Constants.pi;
  final parameter Modelica.SIunits.Area A=pi*(D/2)^2;
  final parameter Modelica.SIunits.Volume V=L*A;

  parameter Modelica.SIunits.Density rho = 1000 "Mass density of fluid";
  parameter Modelica.SIunits.SpecificHeatCapacity cp=4187
    "Specific heat of fluid";

  parameter Boolean dynamicBalance = true
    "Set to true to use a dynamic balance, which often leads to smaller systems of equations"
    annotation (Evaluate=true, Dialog(tab="Dynamics", group="Equations"));
  parameter Modelica.SIunits.ThermalConductivity lambdaI=0.026
    "Heat conductivity";

  parameter Modelica.SIunits.ThermalConductivity R=1/(lambdaI*2*pi/Modelica.Math.log((D/2+h)/(D/2)));
  final parameter Real C=rho*pi*(D/2)^2*cp;

  //Variables
  Real u;
  Modelica.SIunits.Power Q_Losses;

  //Components
  DistrictHeating.Pipes.PlugFlowPipe plugFlowPipe(
    pipeLength=L,
    pipeDiameter=D,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    redeclare package Medium = Medium,
    allowFlowReversal=allowFlowReversal,
    from_dp=from_dp,
    linearizeFlowResistance=linearizeFlowResistance,
    deltaM=deltaM)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));

  Modelica.Blocks.Interfaces.RealInput TBoundary annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,110}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={2,50})));
  TimeDelays.PDETime pDETime
    annotation (Placement(transformation(extent={{-28,24},{-8,44}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=u)
    annotation (Placement(transformation(extent={{-58,24},{-38,44}})));
  BaseClasses.ExponentialDecay tempDecay(C=C, R=R)
    annotation (Placement(transformation(extent={{12,20},{32,40}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort senTem(m_flow_nominal=m_flow_nominal,
      redeclare package Medium = Medium,
    tau=0)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Annex60.Fluid.MixingVolumes.MixingVolume idealHeater(
    m_flow_nominal=m_flow_nominal,
    redeclare package Medium = Medium,
    nPorts=2,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    V=V,
    massDynamics=massDynamics,
    allowFlowReversal=allowFlowReversal)
    annotation (Placement(transformation(extent={{62,0},{82,20}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature1
    annotation (Placement(transformation(extent={{46,4},{58,16}})));
equation
  //Normalized speed of the fluid [1/s]
  u = port_a.m_flow/(rho*V);
  Q_Losses = -idealHeater.heatPort.Q_flow/L;

  connect(port_a, plugFlowPipe.port_a) annotation (Line(
      points={{-100,0},{-60,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pDETime.u, realExpression.y) annotation (Line(
      points={{-30,34},{-37,34}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(plugFlowPipe.port_b, senTem.port_a) annotation (Line(
      points={{-40,0},{-10,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem.T, tempDecay.TIn) annotation (Line(
      points={{0,11},{0,26},{10,26}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pDETime.tau, tempDecay.td) annotation (Line(
      points={{-7,34},{10,34}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TBoundary, tempDecay.Tb) annotation (Line(
      points={{0,110},{0,60},{22,60},{22,42}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(idealHeater.heatPort, prescribedTemperature1.port)
    annotation (Line(points={{62,10},{58,10}}, color={191,0,0}));
  connect(senTem.port_b, idealHeater.ports[1])
    annotation (Line(points={{10,0},{70,0}}, color={0,127,255}));
  connect(idealHeater.ports[2], port_b)
    annotation (Line(points={{74,0},{74,0},{100,0}}, color={0,127,255}));
  connect(tempDecay.TOut, prescribedTemperature1.T) annotation (Line(points={{33,
          30},{40,30},{40,10},{44.8,10}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}),
                   graphics={
        Polygon(
          points={{20,-70},{60,-85},{20,-100},{20,-70}},
          lineColor={0,128,255},
          smooth=Smooth.None,
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid,
          visible=showDesignFlowDirection),
        Polygon(
          points={{20,-75},{50,-85},{20,-95},{20,-75}},
          lineColor={255,255,255},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          visible=allowFlowReversal),
        Polygon(
          points={{20,-75},{50,-85},{20,-95},{20,-75}},
          lineColor={255,255,255},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          visible=allowFlowReversal),
        Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={192,192,192}),
        Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255}),
        Rectangle(
          extent={{-26,30},{30,-30}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Line(
          points={{55,-85},{-60,-85}},
          color={0,128,255},
          smooth=Smooth.None,
          visible=showDesignFlowDirection),
        Rectangle(
          extent={{-100,50},{100,40}},
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-100,-40},{100,-50}},
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Backward)}),
                           Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end PlugFlowHeatLosses;
