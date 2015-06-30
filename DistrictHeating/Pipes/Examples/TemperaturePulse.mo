within DistrictHeating.Pipes.Examples;
model TemperaturePulse
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  IDEAS.Fluid.Sources.FixedBoundary bou2(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-96,0})));

  IDEAS.Fluid.Sources.FixedBoundary bou3(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={88,0})));

  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-24,0})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugOut(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,0})));

  DistrictHeating.Pipes.PlugFlowHeatLosses plug(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    k=0.026,
    L=50)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-96,60},{-76,80}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-36,34},{-28,42}})));
  IDEAS.Fluid.Sources.FixedBoundary bou4(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-96,-40})));
  IDEAS.Fluid.Sources.FixedBoundary bou5(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={88,-40})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-22,-40})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-40})));
  Modelica.Fluid.Pipes.DynamicPipe MSL(
    length=plug.L,
    diameter=plug.D,
    use_HeatTransfer=true,
    redeclare package Medium = Annex60.Media.Water,
    T_start=273.15 + 20,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (T_ambient=273.15 + 20, alpha0=plug.k*plug.S/(plug.pi*plug.D)),
    nNodes=50)
    annotation (Placement(transformation(extent={{-2,-50},{18,-30}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector colAllToOne(m=MSL.nNodes)
    "Connector to assign multiple heat ports to one heat port" annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={8,-26})));
  IDEAS.Fluid.Sources.FixedBoundary bou6(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-96,-80})));
  IDEAS.Fluid.Sources.FixedBoundary bou7(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={88,-80})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TVolIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-22,-80})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TVolOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-80})));
  Annex60.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    nPorts=2,
    V=plug.V)
    annotation (Placement(transformation(extent={{46,-80},{26,-60}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermalResistor1(R=
        plug.r/plug.L) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={46,50})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan1(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-58,-50},{-38,-30}})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan2(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-58,-90},{-38,-70}})));
  Modelica.Blocks.Sources.Pulse pulse(
    width=5,
    period=86400,
    offset=273.15 + 50,
    amplitude=20)
    annotation (Placement(transformation(extent={{-70,-26},{-78,-18}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-84,-10},{-64,10}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater1(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-84,-50},{-64,-30}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater2(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-84,-90},{-64,-70}})));
  Modelica.Blocks.Sources.Pulse pulse1(
    period=86400,
    amplitude=0,
    width=0,
    offset=0.1)
    annotation (Placement(transformation(extent={{-66,-26},{-58,-18}})));
  DistrictHeating.Pipes.PlugFlowHeatLosses plug1(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    k=0.026,
    L=50)
    annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  Modelica.Fluid.Pipes.DynamicPipe MSL1(
    length=plug.L,
    diameter=plug.D,
    use_HeatTransfer=true,
    redeclare package Medium = Annex60.Media.Water,
    T_start=273.15 + 20,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (T_ambient=273.15 + 20, alpha0=plug.k*plug.S/(plug.pi*plug.D)),
    nNodes=50)
    annotation (Placement(transformation(extent={{24,-50},{44,-30}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector colAllToOne1(m=MSL.nNodes)
    "Connector to assign multiple heat ports to one heat port" annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={34,-26})));
  Annex60.Fluid.MixingVolumes.MixingVolume vol1(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    V=plug.V,
    nPorts=2)
    annotation (Placement(transformation(extent={{22,-80},{2,-60}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermalResistor2(R=
        plug.r/plug.L) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={22,52})));
equation
  connect(TPlugOut.port_b, bou3.ports[1]) annotation (Line(
      points={{68,0},{78,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TPlugIn.port_b, plug.port_a) annotation (Line(
      points={{-14,0},{-10,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port, temperatureSensor.port) annotation (Line(
      points={{-76,70},{-62,70},{-62,38},{-36,38}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(temperatureSensor.T, plug.TBoundary) annotation (Line(
      points={{-28,38},{0.2,38},{0.2,5}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TMocOut.port_b, bou5.ports[1]) annotation (Line(
      points={{68,-40},{78,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_b, MSL.port_a) annotation (Line(
      points={{-12,-40},{-2,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(colAllToOne.port_a, MSL.heatPorts) annotation (Line(
      points={{8,-32},{8,-35.6},{8.1,-35.6}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(colAllToOne.port_b, temperatureSensor.port) annotation (Line(
      points={{8,-20},{8,70},{-62,70},{-62,38},{-36,38}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(TVolOut.port_b, bou7.ports[1]) annotation (Line(
      points={{68,-80},{78,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.ports[1], TVolOut.port_a) annotation (Line(
      points={{38,-80},{48,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.heatPort, thermalResistor1.port_b) annotation (Line(
      points={{46,-70},{46,40}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(thermalResistor1.port_a, temperatureSensor.port) annotation (Line(
      points={{46,60},{46,70},{-62,70},{-62,38},{-36,38}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(fan.port_b, TPlugIn.port_a) annotation (Line(
      points={{-38,0},{-34,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_a, fan1.port_b) annotation (Line(
      points={{-32,-40},{-38,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TVolIn.port_a, fan2.port_b) annotation (Line(
      points={{-32,-80},{-38,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou2.ports[1], idealHeater.port_a) annotation (Line(
      points={{-92,0},{-84,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.port_b, fan.port_a) annotation (Line(
      points={{-64,0},{-58,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou4.ports[1], idealHeater1.port_a) annotation (Line(
      points={{-92,-40},{-84,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater1.port_b, fan1.port_a) annotation (Line(
      points={{-64,-40},{-58,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou6.ports[1], idealHeater2.port_a) annotation (Line(
      points={{-92,-80},{-84,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater2.port_b, fan2.port_a) annotation (Line(
      points={{-64,-80},{-58,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pulse.y, idealHeater2.TSet) annotation (Line(
      points={{-78.4,-22},{-90,-22},{-90,-74},{-86,-74}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(idealHeater1.TSet, idealHeater2.TSet) annotation (Line(
      points={{-86,-34},{-90,-34},{-90,-74},{-86,-74}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(idealHeater.TSet, idealHeater2.TSet) annotation (Line(
      points={{-86,6},{-90,6},{-90,-74},{-86,-74}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse1.y, fan1.m_flow_in) annotation (Line(
      points={{-57.6,-22},{-48.2,-22},{-48.2,-28}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse1.y, fan2.m_flow_in) annotation (Line(
      points={{-57.6,-22},{-54,-22},{-54,-64},{-48.2,-64},{-48.2,-68}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(fan.m_flow_in, fan2.m_flow_in) annotation (Line(
      points={{-48.2,12},{-48,12},{-48,18},{-54,18},{-54,-64},{-48.2,-64},{
          -48.2,-68}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(plug.port_b, plug1.port_a) annotation (Line(
      points={{10,0},{18,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug1.port_b, TPlugOut.port_a) annotation (Line(
      points={{38,0},{48,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug1.TBoundary, plug.TBoundary) annotation (Line(
      points={{28.2,5},{28.2,38},{0.2,38},{0.2,5}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(MSL.port_b, MSL1.port_a) annotation (Line(
      points={{18,-40},{24,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(MSL1.port_b, TMocOut.port_a) annotation (Line(
      points={{44,-40},{48,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(MSL1.heatPorts, colAllToOne1.port_a) annotation (Line(
      points={{34.1,-35.6},{34,-35.6},{34,-32}},
      color={127,0,0},
      smooth=Smooth.None));
  connect(colAllToOne1.port_b, temperatureSensor.port) annotation (Line(
      points={{34,-20},{34,-16},{8,-16},{8,70},{-62,70},{-62,38},{-36,38}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(TVolIn.port_b, vol1.ports[1]) annotation (Line(
      points={{-12,-80},{14,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol1.ports[2], vol.ports[2]) annotation (Line(
      points={{10,-80},{34,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(thermalResistor2.port_a, temperatureSensor.port) annotation (Line(
      points={{22,62},{22,70},{-62,70},{-62,38},{-36,38}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(thermalResistor2.port_b, vol1.heatPort) annotation (Line(
      points={{22,42},{22,-70}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=25000),
    __Dymola_experimentSetupOutput,
    __Dymola_Commands(executeCall(ensureSimulated=true) = RunScript(
        "C:/Users/u0098668/Documents/Modelica/DistrictHeating/DistrictHeating/simulate and plot.mos")
        "Simulate and plot"));
end TemperaturePulse;
