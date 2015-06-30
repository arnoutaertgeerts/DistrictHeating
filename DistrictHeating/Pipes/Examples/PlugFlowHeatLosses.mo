within DistrictHeating.Pipes.Examples;
model PlugFlowHeatLosses
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  inner Modelica.Fluid.System system(p_ambient=101325)
                                   annotation (Placement(transformation(extent={{60,70},
            {80,90}},          rotation=0)));

  IDEAS.Fluid.Sources.FixedBoundary bou2(
    nPorts=1,
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,0})));

  IDEAS.Fluid.Sources.FixedBoundary bou3(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,0})));

  IDEAS.Fluid.Movers.Pump pump3(redeclare package Medium =
        IDEAS.Media.Water.Simple, m_flow_nominal=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-58,0})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort Tin(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,0})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort Tout(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,0})));

  DistrictHeating.Pipes.PlugFlowHeatLosses plugFlowHeatLosses(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    L=1000)
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-20,30},{0,50}})));
  Annex60.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    V=plugFlowHeatLosses.V,
    nPorts=1,
    T_start=273.15 + 70)
    annotation (Placement(transformation(extent={{-40,74},{-20,94}})));
  IDEAS.HeatTransfer.ThermalResistor thermalResistor(R=plugFlowHeatLosses.R/(
        plugFlowHeatLosses.V*plugFlowHeatLosses.rho))
                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-46,60})));
  IDEAS.Fluid.Sources.FixedBoundary bou1(
    nPorts=1,
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    use_T=false)
              annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={10,70})));
equation
  connect(pump3.port_b, Tin.port_a) annotation (Line(
      points={{-48,0},{-40,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump3.port_a, bou2.ports[1]) annotation (Line(
      points={{-68,0},{-76,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Tout.port_b, bou3.ports[1]) annotation (Line(
      points={{68,0},{80,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Tin.port_b, plugFlowHeatLosses.port_a) annotation (Line(
      points={{-20,0},{0,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plugFlowHeatLosses.port_b, Tout.port_a) annotation (Line(
      points={{20,0},{48,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port, temperatureSensor.port) annotation (Line(
      points={{-60,40},{-20,40}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(temperatureSensor.T, plugFlowHeatLosses.TBoundary) annotation (Line(
      points={{0,40},{10.2,40},{10.2,5}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(vol.heatPort, thermalResistor.port_b) annotation (Line(
      points={{-40,84},{-46,84},{-46,70}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(thermalResistor.port_a, temperatureSensor.port) annotation (Line(
      points={{-46,50},{-46,40},{-20,40}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(vol.ports[1], bou1.ports[1]) annotation (Line(
      points={{-30,74},{-30,70},{0,70}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=2500),
    __Dymola_experimentSetupOutput);
end PlugFlowHeatLosses;
