within DistrictHeating.Pipes.DoublePipes;
model DHWallenten "District heating pipe based on Wallenten"

  //Extensions
  extends BaseClasses.PartialDistrictHeatingPipe;

  //Parameters
  parameter Integer nSeg=5;

  //Variables
  Modelica.SIunits.Temperature T1;
  Modelica.SIunits.Temperature T2;

  Modelica.SIunits.Temperature Ts "Temperature of the symmetrical problem";
  Modelica.SIunits.Temperature Ta "Temperature of the asymmetrical problem";

  Types.PowerPerLength Qs "Symmetrical heat losses";
  Types.PowerPerLength Qa "Assymmetrical heat losses";

  //Components
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn1(
    redeclare package Medium=Medium,
    tau=tau,
    allowFlowReversal=allowFlowReversal,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-80,50},{-60,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut1(
    redeclare package Medium=Medium,
    tau=tau,
    allowFlowReversal=allowFlowReversal,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{60,50},{80,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut2(
    redeclare package Medium=Medium,
    tau=tau,
    allowFlowReversal=allowFlowReversal,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-60,-70},{-80,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn2(
    redeclare package Medium=Medium,
    tau=tau,
    allowFlowReversal=allowFlowReversal,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{80,-70},{60,-50}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Q2Losses annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-34})));
  Buildings.Fluid.FixedResistances.Pipe      Pipe1(
    redeclare package Medium = Medium,
    massDynamics=massDynamics,
    energyDynamics=energyDynamics,
    length=L,
    diameter=Di,
    lambdaIns=lambdaI,
    nSeg=nSeg,
    allowFlowReversal=allowFlowReversal,
    m_flow_nominal=m_flow_nominal,
    from_dp=from_dp,
    linearizeFlowResistance=linearizeFlowResistance,
    deltaM=deltaM,
    thicknessIns=0.000001,
    dp_nominal=dp_nominal)
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));
  Buildings.Fluid.FixedResistances.Pipe      Pipe2(
    redeclare package Medium = Medium,
    massDynamics=massDynamics,
    energyDynamics=energyDynamics,
    length=L,
    lambdaIns=lambdaI,
    diameter=Di,
    nSeg=nSeg,
    allowFlowReversal=allowFlowReversal,
    m_flow_nominal=m_flow_nominal,
    from_dp=from_dp,
    linearizeFlowResistance=linearizeFlowResistance,
    deltaM=deltaM,
    thicknessIns=0.000001,
    dp_nominal=dp_nominal)
    annotation (Placement(transformation(extent={{10,-70},{-10,-50}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Q1Losses annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,92})));
  Modelica.Blocks.Sources.RealExpression SupplyHeatLosses(y=-Q1)
    annotation (Placement(transformation(extent={{-40,96},{-20,116}})));
  Modelica.Blocks.Sources.RealExpression ReturnHeatLosses(y=-Q2)
    annotation (Placement(transformation(extent={{-40,-22},{-20,-2}})));

equation
  T1 = (TIn1.T + TOut1.T)/2;
  T2 = (TIn2.T + TOut2.T)/2;

  Ts = (T1 + T2)/2;
  Ta = (T1 - T2)/2;

  Qs=(Ts-Tg)*2*Modelica.Constants.pi*lambdaI*hs;
  Qa=Ta*2*Modelica.Constants.pi*lambdaI*ha;

  Q1 = (Qs + Qa)*L;
  Q2 = (Qs - Qa)*L;

  connect(port_a1, TIn1.port_a) annotation (Line(
      points={{-100,60},{-80,60}},
      color={0,127,255},
      smooth=Smooth.None));

  connect(TOut1.port_b, port_b1) annotation (Line(
      points={{80,60},{100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(port_b2, TOut2.port_b) annotation (Line(
      points={{-100,-60},{-80,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TIn2.port_a, port_a2) annotation (Line(
      points={{80,-60},{100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Q2Losses.port, Pipe2.heatPort) annotation (Line(
      points={{0,-44},{0,-55}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(SupplyHeatLosses.y, Q1Losses.Q_flow) annotation (Line(
      points={{-19,106},{0,106},{0,102}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(ReturnHeatLosses.y, Q2Losses.Q_flow) annotation (Line(
      points={{-19,-12},{0,-12},{0,-24}},
      color={0,0,127},
      smooth=Smooth.None));

  connect(TIn1.port_b, Pipe1.port_a) annotation (Line(
      points={{-60,60},{-10,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Pipe1.port_b, TOut1.port_a) annotation (Line(
      points={{10,60},{60,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TOut2.port_a, Pipe2.port_b) annotation (Line(
      points={{-60,-60},{-10,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Pipe2.port_a, TIn2.port_b) annotation (Line(
      points={{10,-60},{60,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Q1Losses.port, Pipe1.heatPort) annotation (Line(
      points={{0,82},{0,65}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},
            {100,140}}), graphics),
                                 Diagram(coordinateSystem(extent={{-100,-140},{
            100,140}},  preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -120},{100,120}}), graphics));
end DHWallenten;
