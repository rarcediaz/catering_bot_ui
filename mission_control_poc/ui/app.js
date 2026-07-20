const state = {
  destinations: [],
  home: null,
  robots: [],
  missions: [],
  socket: null,
  activeScreen: "start",
  operatorPanel: {
    data: null,
    mapPreview: null,
    mapPreviewName: null,
    mapPreviewInFlight: false,
    refreshTimer: null,
    renderedMapKey: null,
    renderedMapCanvas: null,
    frames: {},
    addMapMode: "move",
    locationFilter: "",
    pendingGoal: null,
    mapView: {
      zoom: 1,
      panX: 0,
      panY: 0,
      followRobot: false,
      isPanning: false,
      panPointerId: null,
      startClientX: 0,
      startClientY: 0,
      startPanX: 0,
      startPanY: 0,
      pointerMoved: false,
    },
  },
  returnPromptDismissed: new Set(),
  manualDrive: {
    activeTimer: null,
    currentLinear: 0,
    currentAngular: 0,
    requestInFlight: false,
    pendingCommand: null,
    activeButton: null,
  },
  pointMissionInFlight: false,
};

const MANUAL_BASE_SPEED = 0.5;
const MANUAL_MAX_SPEED = 0.7;
const MANUAL_ACCEL_RATE = 0.1;
const MANUAL_TICK_MS = 100;
const DEFAULT_MAP_NAME = "downstairs_test_july1";
const DASHBOARD_MIN_ZOOM = 0.75;
const DASHBOARD_MAX_ZOOM = 4;
const ROBOT_FOCUS_ZOOM = 1.6;
const ROBOT_FOOTPRINT_FRONT_M = 0.9;
const ROBOT_FOOTPRINT_REAR_M = 0.1;
const ROBOT_FOOTPRINT_WIDTH_M = 0.6;
const MESSAGE_SUCCESS_TIMEOUT_MS = 4500;
const MESSAGE_ERROR_TIMEOUT_MS = 7000;
const messageTimers = new WeakMap();
const DEMO_LOCATIONS = [
  { id: "asb-lab-1", name: "ASB Lab 1", pose: { x: -6.3, y: -2.95, yaw: 0 } },
  { id: "asb-9817", name: "ASB 9817", pose: { x: 0.55, y: -1.25, yaw: 0 } },
  { id: "cs-9813", name: "CS 9813", pose: { x: 8.65, y: -2.15, yaw: 0 } },
  { id: "home-entry", name: "Home Entry", pose: { x: 9.85, y: -5.25, yaw: 0 } },
  { id: "main-entry", name: "Main Entry", pose: { x: 8.95, y: 2.15, yaw: 0 } },
  { id: "storage", name: "Storage", pose: { x: -8.75, y: -1.95, yaw: 0 } },
];

const elements = {
  selectedRobot: document.getElementById("selected-robot"),
  dashboardMapTitle: document.getElementById("demo-map-title"),
  dashboardMapShell: document.getElementById("dashboard-map-shell"),
  destinationOverlay: document.getElementById("destination-overlay"),
  locationSearch: document.getElementById("location-search"),
  locationResults: document.getElementById("location-results"),
  mapZoomIn: document.getElementById("map-zoom-in"),
  mapZoomOut: document.getElementById("map-zoom-out"),
  mapConfirmPopover: document.getElementById("map-confirm-popover"),
  mapConfirmText: document.getElementById("map-confirm-text"),
  confirmMapGo: document.getElementById("confirm-map-go"),
  confirmMapCancel: document.getElementById("confirm-map-cancel"),
  emergencyStopButton: document.getElementById("emergency-stop-button"),
  headerMode: document.getElementById("header-mode"),
  headerBattery: document.getElementById("header-battery"),
  headerConnection: document.getElementById("header-connection"),
  headerLatency: document.getElementById("header-latency"),
  headerMap: document.getElementById("header-map"),

  startRobotId: document.getElementById("start-robot-id"),
  startMode: document.getElementById("start-mode"),
  startBattery: document.getElementById("start-battery"),
  startConnection: document.getElementById("start-connection"),
  startLatency: document.getElementById("start-latency"),
  startMap: document.getElementById("start-map"),
  startLock: document.getElementById("start-lock"),
  destinationsList: document.getElementById("destinations-list"),
  localizeRobotButton: document.getElementById("localize-robot-button"),
  manageMapsButton: document.getElementById("manage-maps-button"),
  startNextButton: document.getElementById("start-next-button"),
  startNextMessage: document.getElementById("start-next-message"),

  manageCurrentMap: document.getElementById("manage-current-map"),
  savedMapSelect: document.getElementById("saved-map-select"),
  selectMapButton: document.getElementById("select-map-button"),
  mappingModeButton: document.getElementById("mapping-mode-button"),
  addDestinationButton: document.getElementById("add-destination-button"),
  manageMessage: document.getElementById("manage-message"),

  mappingStatus: document.getElementById("mapping-status"),
  mappingMapName: document.getElementById("mapping-map-name"),
  startMappingButton: document.getElementById("start-mapping-button"),
  saveMappingButton: document.getElementById("save-mapping-button"),
  mappingMessage: document.getElementById("mapping-message"),
  mappingMapCanvas: document.getElementById("mapping-map-canvas"),
  mappingMapPlaceholder: document.getElementById("mapping-map-placeholder"),
  mappingMapMeta: document.getElementById("mapping-map-meta"),

  addSelectedMap: document.getElementById("add-selected-map"),
  addMapModes: document.getElementById("add-map-modes"),
  addMapCanvas: document.getElementById("add-map-canvas"),
  addMapPlaceholder: document.getElementById("add-map-placeholder"),
  destinationName: document.getElementById("destination-name"),
  destinationX: document.getElementById("destination-x"),
  destinationY: document.getElementById("destination-y"),
  destinationYaw: document.getElementById("destination-yaw"),
  useCurrentPoseButton: document.getElementById("use-current-pose-button"),
  initialPoseX: document.getElementById("initial-pose-x"),
  initialPoseY: document.getElementById("initial-pose-y"),
  initialPoseYaw: document.getElementById("initial-pose-yaw"),
  sendInitialPoseButton: document.getElementById("send-initial-pose-button"),
  saveDestinationButton: document.getElementById("save-destination-button"),
  addDestinationMessage: document.getElementById("add-destination-message"),

  openNewRequestButton: document.getElementById("open-new-request-button"),
  manualModeButton: document.getElementById("manual-mode-button"),
  requestForm: document.getElementById("request-form"),
  requester: document.getElementById("requester"),
  operatorId: document.getElementById("operator-id"),
  requestDestination: document.getElementById("request-destination"),
  tripType: document.getElementById("trip-type"),
  returnDestinationField: document.getElementById("return-destination-field"),
  returnDestination: document.getElementById("return-destination"),
  requestRobot: document.getElementById("request-robot"),
  requestNotes: document.getElementById("request-notes"),
  createRequestButton: document.getElementById("create-request-button"),
  createAnotherButton: document.getElementById("create-another-button"),
  goStateButton: document.getElementById("go-state-button"),
  requestMessage: document.getElementById("request-message"),

  pendingRequestsList: document.getElementById("pending-requests-list"),
  stateMessage: document.getElementById("state-message"),
  clearPendingButton: document.getElementById("clear-pending-button"),
  cancelCurrentMissionButton: document.getElementById("cancel-current-mission-button"),
  activeMissionCard: document.getElementById("active-mission-card"),
  stateRobotId: document.getElementById("state-robot-id"),
  stateBattery: document.getElementById("state-battery"),
  stateMode: document.getElementById("state-mode"),
  stateConnection: document.getElementById("state-connection"),
  stateLatency: document.getElementById("state-latency"),
  stateLock: document.getElementById("state-lock"),
  stateWarning: document.getElementById("state-warning"),
  stateMapCanvas: document.getElementById("state-map-canvas"),
  stateMapPlaceholder: document.getElementById("state-map-placeholder"),
  stateMapMeta: document.getElementById("state-map-meta"),
  saveTempDestinationButton: document.getElementById("save-temp-destination-button"),
  mapMessage: document.getElementById("map-message"),
  manualDriveShell: document.getElementById("manual-drive-shell"),
  manualControlPanel: document.getElementById("manual-control-panel"),
  manualPad: document.getElementById("manual-pad"),
  manualStatus: document.getElementById("manual-status"),
  manualMessage: document.getElementById("manual-message"),
  clearCompletedButton: document.getElementById("clear-completed-button"),
  clearAllButton: document.getElementById("clear-all-button"),
  queueMessage: document.getElementById("queue-message"),
  missionsBody: document.getElementById("missions-body"),
  returnModal: document.getElementById("return-modal"),
  returnModalText: document.getElementById("return-modal-text"),
  returnModalButton: document.getElementById("return-modal-button"),
  returnModalStayButton: document.getElementById("return-modal-stay-button"),
  calibrationStatusDot: document.getElementById("calibration-status-dot"),
  calibrationLocalization: document.getElementById("calibration-localization"),
  calibrationInitialPose: document.getElementById("calibration-initial-pose"),
  calibrationGoalPose: document.getElementById("calibration-goal-pose"),
  calibrationUseCurrentPoseButton: document.getElementById("calibration-use-current-pose-button"),
  calibrationPoseX: document.getElementById("calibration-pose-x"),
  calibrationPoseY: document.getElementById("calibration-pose-y"),
  calibrationPoseYaw: document.getElementById("calibration-pose-yaw"),
  calibrationSendInitialPoseButton: document.getElementById("calibration-send-initial-pose-button"),
  calibrationMessage: document.getElementById("calibration-message"),
};

document.addEventListener("DOMContentLoaded", () => {
  document.querySelectorAll("[data-nav-screen]").forEach((button) => {
    button.addEventListener("click", () => showScreen(button.dataset.navScreen));
  });

  elements.selectedRobot.addEventListener("change", async () => {
    state.operatorPanel.data = null;
    const robot = getSelectedRobot();
    if (isRobotVisibleOnMap(robot)) {
      enableRobotFollow();
    } else {
      resetDashboardMapView();
    }
    clearPendingMapGoal();
    state.operatorPanel.frames = {};
    await loadOperatorPanel();
    await loadDashboardMapPreview({ silent: true });
    renderAll();
  });

  elements.manageMapsButton.addEventListener("click", () => showScreen("manage"));
  elements.localizeRobotButton.addEventListener("click", handleLocalizeRobot);
  elements.locationSearch.addEventListener("input", () => {
    state.operatorPanel.locationFilter = elements.locationSearch.value.trim().toLowerCase();
    renderLocationResults();
  });
  elements.locationResults.addEventListener("click", handleLocationClick);
  elements.destinationOverlay.addEventListener("click", handleLocationClick);
  elements.mapZoomIn.addEventListener("click", () => zoomDashboardMap(1.25));
  elements.mapZoomOut.addEventListener("click", () => zoomDashboardMap(0.8));
  elements.confirmMapGo.addEventListener("click", executePendingMapGoal);
  elements.confirmMapCancel.addEventListener("click", clearPendingMapGoal);
  elements.calibrationUseCurrentPoseButton.addEventListener("click", fillCalibrationPoseFromRobot);
  elements.calibrationSendInitialPoseButton.addEventListener("click", handleSendCalibrationInitialPose);
  elements.startNextButton.addEventListener("click", handleStartNext);
  elements.selectMapButton.addEventListener("click", handleSelectMap);
  elements.savedMapSelect.addEventListener("change", () => {
    if (elements.savedMapSelect.value) {
      void handleSelectMap();
    }
  });
  elements.mappingModeButton.addEventListener("click", () => showScreen("mapping"));
  elements.addDestinationButton.addEventListener("click", handleOpenAddDestination);
  elements.startMappingButton.addEventListener("click", handleStartMapping);
  elements.saveMappingButton.addEventListener("click", handleSaveMapping);
  elements.addMapModes.addEventListener("click", handleAddMapMode);
  elements.addMapCanvas.addEventListener("click", handleAddMapClick);
  elements.useCurrentPoseButton.addEventListener("click", fillInitialPoseFromRobot);
  elements.sendInitialPoseButton.addEventListener("click", handleSendInitialPose);
  elements.saveDestinationButton.addEventListener("click", handleSaveDestination);
  elements.openNewRequestButton.addEventListener("click", () => showScreen("new-request"));
  elements.manualModeButton.addEventListener("click", toggleManualControls);
  elements.tripType.addEventListener("change", syncTripType);
  elements.requestForm.addEventListener("submit", handleCreateRequest);
  elements.createAnotherButton.addEventListener("click", resetRequestForm);
  elements.goStateButton.addEventListener("click", () => showScreen("state"));
  elements.pendingRequestsList.addEventListener("click", handleStartRequestClick);
  elements.clearPendingButton.addEventListener("click", () =>
    handleQueueReset({
      button: elements.clearPendingButton,
      endpoint: "/admin/requests/clear-pending",
      confirmText: "Clear all pending requests?",
      successLabel: "Pending requests cleared",
      messageTarget: elements.stateMessage,
    })
  );
  elements.cancelCurrentMissionButton.addEventListener("click", handleCancelCurrentMission);
  elements.activeMissionCard.addEventListener("click", handleMissionAction);
  elements.missionsBody.addEventListener("click", handleMissionAction);
  elements.returnModalButton.addEventListener("click", handleMissionAction);
  elements.returnModalStayButton.addEventListener("click", handleReturnStay);
  elements.saveTempDestinationButton.addEventListener("click", handleSaveTempDestination);
  elements.stateMapCanvas.addEventListener("click", handleDashboardMapClick);
  elements.stateMapCanvas.addEventListener("pointerdown", handleDashboardMapPointerDown);
  elements.stateMapCanvas.addEventListener("pointermove", handleDashboardMapPointerMove);
  elements.stateMapCanvas.addEventListener("pointerup", handleDashboardMapPointerUp);
  elements.stateMapCanvas.addEventListener("pointercancel", handleDashboardMapPointerUp);
  elements.stateMapCanvas.addEventListener("contextmenu", (event) => event.preventDefault());
  elements.clearCompletedButton.addEventListener("click", () =>
    handleQueueReset({
      button: elements.clearCompletedButton,
      endpoint: "/admin/missions/clear-completed",
      confirmText: "Clear completed mission history?",
      successLabel: "Completed missions cleared",
    })
  );
  elements.clearAllButton.addEventListener("click", () =>
    handleQueueReset({
      button: elements.clearAllButton,
      endpoint: "/admin/missions/clear-all",
      confirmText: "Clear the started mission queue and history? Pending requests are separate.",
      successLabel: "Queue and history cleared",
    })
  );

  document.body.addEventListener("click", handlePowerAction);
  elements.manualPad.addEventListener("pointerdown", handleManualPadPointerDown);

  window.addEventListener("pointerup", () => stopManualDrive());
  window.addEventListener("pointercancel", () => stopManualDrive());
  window.addEventListener("blur", () => stopManualDrive({ silent: true }));
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      stopManualDrive({ silent: true });
    }
  });

  void boot();
});

async function boot() {
  await Promise.all([loadDestinations(), loadSnapshot()]);
  await loadOperatorPanel();
  await loadDashboardMapPreview({ silent: true });
  startOperatorPanelRefresh();
  connectStatusStream();
  syncTripType();
  renderAll();
}

async function loadDestinations() {
  const response = await fetch("/destinations");
  const payload = await response.json();
  state.destinations = payload.destinations ?? [];
  state.home = payload.home ?? null;
  populateDestinationSelects();
  renderDestinations();
}

async function loadSnapshot() {
  const response = await fetch("/status");
  const payload = await response.json();
  applySnapshot(payload);
}

function connectStatusStream() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(`${protocol}://${window.location.host}/ws/status`);
  state.socket = socket;
  socket.addEventListener("open", () => renderConnectionState(true));
  socket.addEventListener("message", (event) => applySnapshot(JSON.parse(event.data)));
  socket.addEventListener("close", () => {
    renderConnectionState(false);
    window.setTimeout(connectStatusStream, 2000);
  });
  socket.addEventListener("error", () => socket.close());
}

function applySnapshot(snapshot) {
  state.robots = snapshot.robots ?? [];
  state.missions = snapshot.missions ?? [];
  populateRobotSelects();
  renderAll();
}

function startOperatorPanelRefresh() {
  if (state.operatorPanel.refreshTimer !== null) {
    window.clearInterval(state.operatorPanel.refreshTimer);
  }
  state.operatorPanel.refreshTimer = window.setInterval(async () => {
    await loadOperatorPanel({ silent: true });
    await loadDashboardMapPreview({ silent: true });
  }, 3000);
}

async function loadOperatorPanel({ silent = false } = {}) {
  const robot = getMapContextRobot();
  if (!robot) {
    state.operatorPanel.data = null;
    renderAll();
    return;
  }

  try {
    const response = await fetch(`/robots/${encodeURIComponent(robot.id)}/operator-panel`);
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.detail || "Robot panel load failed");
    }
    state.operatorPanel.data = payload;
  } catch (error) {
    if (!silent) {
      setMessage(elements.stateMessage, error.message || "Robot panel load failed", true);
    }
    state.operatorPanel.data = null;
  } finally {
    renderAll();
  }
}

async function loadDashboardMapPreview({ silent = false } = {}) {
  const robot = getMapContextRobot();
  if (!robot || state.operatorPanel.mapPreviewInFlight) {
    return;
  }
  const mapName = currentMapName() || getSavedMaps()[0] || DEFAULT_MAP_NAME;
  if (state.operatorPanel.mapPreviewName === mapName && state.operatorPanel.mapPreview) {
    return;
  }

  state.operatorPanel.mapPreviewInFlight = true;
  try {
    const response = await fetch(
      `/robots/${encodeURIComponent(robot.id)}/maps/${encodeURIComponent(mapName)}/preview`
    );
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.detail || "Map preview load failed.");
    }
    state.operatorPanel.mapPreview = payload.map;
    state.operatorPanel.mapPreviewName = mapName;
    resetDashboardMapView({ render: false });
  } catch (error) {
    if (!silent) {
      setMessage(elements.mapMessage, error.message || "Map preview load failed.", true);
    }
  } finally {
    state.operatorPanel.mapPreviewInFlight = false;
    renderAll();
  }
}

function renderAll() {
  renderHeader();
  renderStartRobotState();
  renderMapSetup();
  renderStateRobotState();
  renderCalibrationPanel();
  renderPendingRequests();
  renderActiveMission();
  renderMissions();
  renderReturnPrompt();
  renderManualAvailability();
  renderMaps();
  renderLocationResults();
  syncTripType();
  highlightNav();
}

function showScreen(name) {
  state.activeScreen = name || "start";
  document.querySelectorAll(".screen").forEach((screen) => {
    screen.classList.toggle("hidden", screen.id !== `screen-${state.activeScreen}`);
  });
  if (state.activeScreen === "add-destination") {
    renderMap("add");
  }
  if (state.activeScreen === "mapping") {
    renderMap("mapping");
  }
  if (state.activeScreen === "state") {
    renderMap("state");
    renderReturnPrompt();
  }
  highlightNav();
}

function highlightNav() {
  document.querySelectorAll(".top-nav [data-nav-screen]").forEach((button) => {
    button.classList.toggle("active", button.dataset.navScreen === state.activeScreen);
  });
}

function renderConnectionState(isConnected) {
  setText(elements.headerConnection, isConnected ? "Live" : "Reconnecting");
}

function populateDestinationSelects() {
  const destinationOptions = state.destinations
    .map((destination) => `<option value="${escapeHtml(destination.name)}">${escapeHtml(destination.name)}</option>`)
    .join("");
  elements.requestDestination.innerHTML = destinationOptions;
  elements.returnDestination.innerHTML =
    `<option value="">${state.home ? `Home (${escapeHtml(state.home)})` : "Use Home"}</option>` + destinationOptions;
}

function populateRobotSelects() {
  const selectedRobotId = elements.selectedRobot.value;
  const requestRobotId = elements.requestRobot.value;
  const robots = sortedRobotsForSelection();
  const robotOptions = robots
    .map((robot) => {
      const suffix = robotConnectionLabel(robot) === "Connected" ? "" : " (offline)";
      return `<option value="${escapeHtml(robot.id)}">${escapeHtml(robot.id + suffix)}</option>`;
    })
    .join("");

  elements.selectedRobot.innerHTML = '<option value="">Map overview</option>' + robotOptions;
  if ([...elements.selectedRobot.options].some((option) => option.value === selectedRobotId)) {
    elements.selectedRobot.value = selectedRobotId;
  }

  elements.requestRobot.innerHTML = '<option value="">Auto-select available robot</option>' + robotOptions;
  if ([...elements.requestRobot.options].some((option) => option.value === requestRobotId)) {
    elements.requestRobot.value = requestRobotId;
  }
}

function renderHeader() {
  const robot = getSelectedRobot();
  const power = robot?.power ?? {};
  const mode = displayPowerMode(power.mode || (robot?.mode === "ManualOverride" ? "MANUAL" : "AUTO"));
  const battery = power.battery_percent ?? batteryPercentFromVoltage(robot?.battery_v);
  const mapName = dashboardMapDisplayName();

  setText(elements.headerMode, robot ? mode : "--");
  setText(elements.headerBattery, battery == null ? "--" : `${formatNumber(battery)}%`);
  setText(elements.headerConnection, robot ? robotConnectionLabel(robot) : "--");
  setText(elements.headerLatency, power.latency_ms == null ? "--" : `${formatNumber(power.latency_ms)} ms`);
  setText(elements.headerMap, mapName || "No map");
  setText(elements.dashboardMapTitle, mapName || "No map selected");
}

function renderStartRobotState() {
  const robot = getSelectedRobot();
  const power = robot?.power ?? {};
  const battery = power.battery_percent ?? batteryPercentFromVoltage(robot?.battery_v);
  const mode = displayPowerMode(power.mode || (robot?.mode === "ManualOverride" ? "MANUAL" : "AUTO"));

  setText(elements.startRobotId, robot?.id || "--");
  setText(elements.startMode, robot ? mode : "--");
  setText(elements.startBattery, battery == null ? "--" : `${formatNumber(battery)}%`);
  setText(elements.startConnection, robot ? robotConnectionLabel(robot) : "--");
  setText(elements.startLatency, power.latency_ms == null ? "--" : `${formatNumber(power.latency_ms)} ms`);
  setText(elements.startMap, dashboardMapDisplayName() || "No map");
  setText(elements.startLock, robot ? (power.safety_lock ? "Locked" : "Ready") : "--");
}

function renderStateRobotState() {
  const robot = getSelectedRobot();
  const power = robot?.power ?? {};
  const battery = power.battery_percent ?? batteryPercentFromVoltage(robot?.battery_v);
  const mode = displayPowerMode(power.mode || (robot?.mode === "ManualOverride" ? "MANUAL" : "AUTO"));

  setText(elements.stateRobotId, robot?.id || "--");
  setText(elements.stateBattery, battery == null ? "--" : `${formatNumber(battery)}%`);
  setText(elements.stateMode, robot ? mode : "--");
  setText(elements.stateConnection, robot ? robotConnectionLabel(robot) : "--");
  setText(elements.stateLatency, power.latency_ms == null ? "--" : `${formatNumber(power.latency_ms)} ms`);
  setText(elements.stateLock, robot ? (power.safety_lock ? "Locked" : "Ready") : "--");
  setText(elements.stateWarning, robot ? robotWarningLabel(robot, battery) : "--");
}

function renderCalibrationPanel() {
  const robot = getSelectedRobot();
  const data = state.operatorPanel.data;
  const initialPose = data?.initial_pose;
  const goalPose = data?.goal_pose;
  const localizationValid = Boolean(robot?.localization_valid || initialPose);
  const connected = robot && robotConnectionLabel(robot) === "Connected";

  setText(elements.calibrationLocalization, robot ? (localizationValid ? "Ready" : "Not confirmed") : "--");
  setText(elements.calibrationInitialPose, initialPose ? formatPoseShort(initialPose) : "Not set");
  setText(elements.calibrationGoalPose, goalPose ? formatPoseShort(goalPose) : "No goal");

  if (elements.calibrationStatusDot) {
    elements.calibrationStatusDot.classList.remove("ready", "warning", "error");
    if (!robot || !connected) {
      elements.calibrationStatusDot.classList.add("error");
    } else if (!localizationValid) {
      elements.calibrationStatusDot.classList.add("warning");
    } else {
      elements.calibrationStatusDot.classList.add("ready");
    }
  }
}

function renderDestinations() {
  if (!state.destinations.length) {
    elements.destinationsList.innerHTML = '<p class="empty-state">No destinations configured.</p>';
    return;
  }

  elements.destinationsList.innerHTML = state.destinations
    .map((destination) => {
      const pose = destination.pose ?? {};
      return `
        <div class="destination-row">
          <strong>${escapeHtml(destination.name)}</strong>
          <span>x ${formatNumber(pose.x)}, y ${formatNumber(pose.y)}, yaw ${formatNumber(pose.yaw)}</span>
          <span class="muted">${escapeHtml(destination.notes || "")}</span>
        </div>
      `;
    })
    .join("");
}

function renderMapSetup() {
  const savedMaps = getSavedMaps();
  const current = currentMapName();
  const previousValue = elements.savedMapSelect.value;
  elements.manageCurrentMap.textContent = `Current Map: ${dashboardMapDisplayName() || current || "No map selected"}`;
  elements.addSelectedMap.textContent = current || "No map selected";

  elements.savedMapSelect.innerHTML =
    savedMaps.length
      ? savedMaps.map((name) => `<option value="${escapeHtml(name)}">${escapeHtml(name)}</option>`).join("")
      : '<option value="">No saved maps found</option>';

  if (savedMaps.includes(previousValue)) {
    elements.savedMapSelect.value = previousValue;
  } else if (current && savedMaps.includes(current)) {
    elements.savedMapSelect.value = current;
  }
  elements.selectMapButton.disabled = !elements.savedMapSelect.value;
}

function renderPendingRequests() {
  const requests = getPendingRequests();
  if (!requests.length) {
    elements.pendingRequestsList.innerHTML = '<p class="empty-state">No pending requests.</p>';
    return;
  }

  elements.pendingRequestsList.innerHTML = requests
    .map((request) => {
      const route = formatRoute(request);
      return `
        <div class="request-row">
          <strong>${formatRequestNumber(request)}</strong>
          <span>Destination: ${escapeHtml(route)}</span>
          <span>Robot: ${escapeHtml(request.assigned_robot_id || "Auto")}</span>
          <button class="primary-button" type="button" data-start-request="${escapeHtml(request.id)}">Start Mission</button>
        </div>
      `;
    })
    .join("");
}

function renderActiveMission() {
  const mission = state.missions.find((item) =>
    item.state !== "Requested" &&
    item.state !== "Completed" &&
    item.outcome !== "Canceled" &&
    item.outcome !== "Failed" &&
    item.outcome !== "Aborted"
  );

  if (!mission) {
    elements.activeMissionCard.innerHTML = '<p class="empty-state">No active mission.</p>';
    elements.cancelCurrentMissionButton.removeAttribute("data-action");
    elements.cancelCurrentMissionButton.removeAttribute("data-mission-id");
    return;
  }

  const canCancel = mission.state !== "Completed";
  if (canCancel) {
    elements.cancelCurrentMissionButton.dataset.action = "cancel";
    elements.cancelCurrentMissionButton.dataset.missionId = mission.id;
  } else {
    elements.cancelCurrentMissionButton.removeAttribute("data-action");
    elements.cancelCurrentMissionButton.removeAttribute("data-mission-id");
  }

  const returnDestination = mission.from_dest || state.home || "Home";
  const returnCallout =
    mission.state === "WaitingForReturn"
      ? `
        <div class="return-callout">
          <strong>Arrived at ${escapeHtml(mission.to_dest)}.</strong>
          <span>The robot is waiting at the destination.</span>
          <div class="mission-actions">
            <button class="action-button return" type="button" data-action="return" data-mission-id="${escapeHtml(mission.id)}">Return to ${escapeHtml(returnDestination)}</button>
          </div>
        </div>
      `
      : "";

  elements.activeMissionCard.innerHTML = `
    <div class="mission-summary">
      <strong>Mission from ${formatRequestNumber(mission)}</strong>
      <span>Destination: ${escapeHtml(formatRoute(mission))}</span>
      <span>Robot: ${escapeHtml(mission.assigned_robot_id || "Auto")}</span>
      <span>State: ${escapeHtml(displayMissionStatus(mission))}</span>
      <span>Requester: ${escapeHtml(mission.requested_by || "--")}</span>
    </div>
    ${returnCallout}
    <div class="mission-actions">
      ${buildMissionActionButton("pause", mission)}
      ${buildMissionActionButton("resume", mission)}
      ${buildMissionActionButton("cancel", mission)}
    </div>
  `;
}

function renderMissions() {
  const startedMissions = state.missions.filter((mission) => mission.state !== "Requested");
  if (!startedMissions.length) {
    elements.missionsBody.innerHTML = '<tr><td colspan="5" class="empty-state">No started missions yet.</td></tr>';
    return;
  }

  elements.missionsBody.innerHTML = startedMissions
    .map((mission) => `
      <tr>
        <td><span class="tag ${slugify(displayMissionStatus(mission))}">${escapeHtml(displayMissionStatus(mission))}</span></td>
        <td>
          <strong>${escapeHtml(formatRoute(mission))}</strong>
          <span class="muted">${formatRequestNumber(mission)}</span>
        </td>
        <td>${escapeHtml(mission.assigned_robot_id || "Auto")}</td>
        <td>${escapeHtml(mission.requested_by || "--")}</td>
        <td><div class="mission-actions">${buildMissionActionButton("return", mission)}${buildMissionActionButton("pause", mission)}${buildMissionActionButton("resume", mission)}${buildMissionActionButton("cancel", mission)}</div></td>
      </tr>
    `)
    .join("");
}

function renderReturnPrompt() {
  const mission = getWaitingForReturnMission();
  if (
    !mission ||
    state.activeScreen !== "state" ||
    state.returnPromptDismissed.has(mission.id)
  ) {
    hideReturnModal();
    return;
  }

  const returnDestination = mission.from_dest || state.home || "Home";
  elements.returnModalText.textContent = `The robot arrived at ${mission.to_dest} and is waiting. Click Return when it should go back to ${returnDestination}.`;
  elements.returnModalButton.textContent = `Return to ${returnDestination}`;
  elements.returnModalButton.dataset.action = "return";
  elements.returnModalButton.dataset.missionId = mission.id;
  elements.returnModal.classList.remove("hidden");
}

function hideReturnModal() {
  elements.returnModal.classList.add("hidden");
  elements.returnModalButton.removeAttribute("data-action");
  elements.returnModalButton.removeAttribute("data-mission-id");
}

function handleReturnStay() {
  const mission = getWaitingForReturnMission();
  if (mission) {
    state.returnPromptDismissed.add(mission.id);
  }
  hideReturnModal();
}

function renderManualAvailability() {
  const robot = getSelectedRobot();
  const buttons = [...elements.manualPad.querySelectorAll("[data-manual-linear], [data-manual-stop]")];
  const available = isManualDriveAvailable(robot);
  elements.manualDriveShell.classList.toggle("is-disabled", !available);
  buttons.forEach((button) => {
    button.disabled = !available;
    if (!available) {
      button.classList.remove("is-active");
    }
  });
  elements.manualStatus.textContent = available
    ? `Manual drive ready for ${robot.id}.`
    : "Manual driving is available when the robot is on.";
  if (!available) {
    stopManualDrive({ silent: true });
  }
}

function renderMaps() {
  renderMap("mapping");
  renderMap("add");
  renderMap("state");
}

function renderMap(kind) {
  const config = getMapConfig(kind);
  if (!config) {
    return;
  }
  const data = state.operatorPanel.data;
  const map = getMapForKind(kind);
  const robot = getSelectedRobot();

  if (!map) {
    config.placeholder.classList.remove("hidden");
    config.placeholder.textContent =
      kind === "state"
        ? `Loading ${dashboardMapDisplayName() || "map"}.`
        : currentMapName()
          ? "Waiting for live map data."
          : "Select a map to show the live map.";
    setText(config.meta, "No live map yet");
    clearCanvas(config.canvas);
    if (kind === "state") {
      renderDestinationOverlay();
    }
    return;
  }

  config.placeholder.classList.add("hidden");
  const mapLabel = map.name || currentMapName() || "Live map";
  setText(config.meta, `${mapLabel}: ${map.width} x ${map.height}, ${formatNumber(map.resolution)} m/cell`);
  drawMap(config.canvas, map, {
    robot: isRobotVisibleOnMap(robot) ? robot : null,
    initialPose: data?.initial_pose,
    goalPose: data?.goal_pose,
    pendingGoal: kind === "state" ? state.operatorPanel.pendingGoal?.pose : null,
    view: kind === "state" ? state.operatorPanel.mapView : null,
  });
  if (kind === "state") {
    renderDestinationOverlay();
    renderMapConfirmPopover();
  }
}

function getMapForKind(kind) {
  const data = state.operatorPanel.data;
  if (data?.map_available && data.map) {
    return data.map;
  }
  if (kind === "state") {
    return state.operatorPanel.mapPreview;
  }
  return null;
}

function getDashboardLocations() {
  const locationsByName = new Map();
  DEMO_LOCATIONS.forEach((location) => {
    locationsByName.set(location.name.toLowerCase(), {
      ...location,
      source: "demo",
      configuredDestination: false,
    });
  });
  state.destinations.forEach((destination) => {
    const pose = destination.pose || {};
    if ([pose.x, pose.y].some((value) => value == null || Number.isNaN(Number(value)))) {
      return;
    }
    const key = destination.name.toLowerCase();
    if (locationsByName.has(key)) {
      return;
    }
    locationsByName.set(key, {
      id: `destination-${slugify(destination.name)}`,
      name: destination.name,
      pose: {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: Number(pose.yaw || 0),
      },
      source: "configured",
      configuredDestination: true,
    });
  });
  return [...locationsByName.values()].sort((a, b) => a.name.localeCompare(b.name));
}

function renderLocationResults() {
  if (!elements.locationResults) {
    return;
  }
  const filter = state.operatorPanel.locationFilter;
  const locations = getDashboardLocations().filter((location) =>
    !filter || location.name.toLowerCase().includes(filter)
  );
  if (!locations.length) {
    elements.locationResults.innerHTML = '<p class="empty-state">No locations found.</p>';
    return;
  }

  elements.locationResults.innerHTML = locations
    .map((location) => `
      <button class="location-result" type="button" data-location-id="${escapeHtml(location.id)}">
        <span>${escapeHtml(location.name)}</span>
      </button>
    `)
    .join("");
}

function renderDestinationOverlay() {
  if (!elements.destinationOverlay) {
    return;
  }
  const frame = state.operatorPanel.frames[elements.stateMapCanvas.id];
  if (!frame) {
    elements.destinationOverlay.innerHTML = "";
    return;
  }

  elements.destinationOverlay.innerHTML = getDashboardLocations()
    .map((location) => {
      const point = worldToCanvasWithFrame(location.pose, frame);
      if (
        !isPointWithinFrame(point, frame) ||
        point.x < 0 ||
        point.x > elements.stateMapCanvas.width ||
        point.y < 0 ||
        point.y > elements.stateMapCanvas.height
      ) {
        return "";
      }
      const left = (point.x / elements.stateMapCanvas.width) * 100;
      const top = (point.y / elements.stateMapCanvas.height) * 100;
      return `
        <button
          class="map-destination-chip"
          type="button"
          data-location-id="${escapeHtml(location.id)}"
          style="left:${left.toFixed(3)}%;top:${top.toFixed(3)}%;"
        >${escapeHtml(location.name)}</button>
      `;
    })
    .join("");
}

function handleLocationClick(event) {
  const button = event.target.closest("[data-location-id]");
  if (!button) {
    return;
  }
  event.preventDefault();
  const location = getDashboardLocations().find((item) => item.id === button.dataset.locationId);
  if (!location) {
    return;
  }
  promptMapGoal({
    name: location.name,
    pose: location.pose,
    configuredDestination: location.configuredDestination,
  });
}

function getMapConfig(kind) {
  const configs = {
    mapping: {
      canvas: elements.mappingMapCanvas,
      placeholder: elements.mappingMapPlaceholder,
      meta: elements.mappingMapMeta,
    },
    add: {
      canvas: elements.addMapCanvas,
      placeholder: elements.addMapPlaceholder,
      meta: null,
    },
    state: {
      canvas: elements.stateMapCanvas,
      placeholder: elements.stateMapPlaceholder,
      meta: elements.stateMapMeta,
    },
  };
  return configs[kind] || null;
}

function zoomDashboardMap(factor) {
  const view = state.operatorPanel.mapView;
  const nextZoom = Math.max(DASHBOARD_MIN_ZOOM, Math.min(DASHBOARD_MAX_ZOOM, view.zoom * factor));
  if (Math.abs(nextZoom - view.zoom) < 0.001) {
    return;
  }
  view.zoom = nextZoom;
  renderMap("state");
}

function enableRobotFollow({ render = true } = {}) {
  const view = state.operatorPanel.mapView;
  view.zoom = ROBOT_FOCUS_ZOOM;
  view.followRobot = true;
  view.pointerMoved = false;
  if (render) {
    renderMap("state");
  }
}

function resetDashboardMapView({ render = true } = {}) {
  const view = state.operatorPanel.mapView;
  view.zoom = 1;
  view.panX = 0;
  view.panY = 0;
  view.followRobot = false;
  view.pointerMoved = false;
  if (render) {
    renderMap("state");
  }
}

function handleDashboardMapPointerDown(event) {
  if (state.activeScreen !== "start" || event.button !== 2) {
    return;
  }
  event.preventDefault();
  const view = state.operatorPanel.mapView;
  view.isPanning = true;
  view.panPointerId = event.pointerId;
  view.startClientX = event.clientX;
  view.startClientY = event.clientY;
  view.startPanX = view.panX;
  view.startPanY = view.panY;
  view.pointerMoved = false;
  elements.stateMapCanvas.setPointerCapture(event.pointerId);
  elements.stateMapCanvas.classList.add("is-panning");
}

function handleDashboardMapPointerMove(event) {
  const view = state.operatorPanel.mapView;
  if (!view.isPanning || event.pointerId !== view.panPointerId) {
    return;
  }
  const rect = elements.stateMapCanvas.getBoundingClientRect();
  const dx = (event.clientX - view.startClientX) * (elements.stateMapCanvas.width / rect.width);
  const dy = (event.clientY - view.startClientY) * (elements.stateMapCanvas.height / rect.height);
  if (Math.abs(dx) > 2 || Math.abs(dy) > 2) {
    view.pointerMoved = true;
    view.followRobot = false;
  }
  view.panX = view.startPanX + dx;
  view.panY = view.startPanY + dy;
  renderMap("state");
}

function handleDashboardMapPointerUp(event) {
  const view = state.operatorPanel.mapView;
  if (!view.isPanning || event.pointerId !== view.panPointerId) {
    return;
  }
  view.isPanning = false;
  view.panPointerId = null;
  if (elements.stateMapCanvas.hasPointerCapture(event.pointerId)) {
    elements.stateMapCanvas.releasePointerCapture(event.pointerId);
  }
  elements.stateMapCanvas.classList.remove("is-panning");
  window.setTimeout(() => {
    view.pointerMoved = false;
  }, 0);
}

async function handleLocalizeRobot() {
  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.startNextMessage, "Select a robot before localizing.", true);
    return;
  }

  elements.localizeRobotButton.disabled = true;
  setMessage(elements.startNextMessage, "Localizing robot. It will rotate briefly.", false);
  try {
    const response = await fetch(`/robots/${encodeURIComponent(robot.id)}/localize`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command_source: getCommandSource() }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Localization failed.");
    }
    setMessage(elements.startNextMessage, body.localization?.message || "Localization complete.", false);
    await loadSnapshot();
    await loadOperatorPanel({ silent: true });
  } catch (error) {
    setMessage(elements.startNextMessage, error.message || "Localization failed.", true);
  } finally {
    elements.localizeRobotButton.disabled = false;
  }
}

function handleStartNext() {
  if (!currentMapName()) {
    setMessage(elements.startNextMessage, "Please create or select a map before creating a request.", true);
    return;
  }
  setMessage(elements.startNextMessage, "", false);
  showScreen("assign");
}

function toggleManualControls() {
  elements.manualControlPanel.classList.toggle("hidden");
  renderManualAvailability();
}

async function handleSelectMap() {
  const mapName = elements.savedMapSelect.value;
  if (!mapName) {
    setMessage(elements.manageMessage, "Choose a saved map first.", true);
    return;
  }
  try {
    await sendSystemCommand("launch_nav", { mapName, messageTarget: elements.manageMessage });
    setMessage(elements.manageMessage, `Selected map: ${mapName}`, false);
    await loadOperatorPanel({ silent: true });
    await loadDashboardMapPreview({ silent: true });
  } catch (error) {
    setMessage(elements.manageMessage, error.message || "Map selection failed.", true);
  }
}

function handleOpenAddDestination() {
  const savedMaps = getSavedMaps();
  if (!savedMaps.length) {
    setMessage(elements.mappingMessage, "A map is required before adding destinations. Please create or select a map first.", true);
    showScreen("mapping");
    return;
  }
  if (!currentMapName()) {
    setMessage(elements.manageMessage, "Select a saved map before adding a destination.", true);
    return;
  }
  setMessage(elements.addDestinationMessage, "", false);
  showScreen("add-destination");
}

async function handleStartMapping() {
  try {
    await sendSystemCommand("launch_slam", { messageTarget: elements.mappingMessage });
    elements.mappingStatus.textContent = "Mapping mode started";
    setMessage(elements.mappingMessage, "Mapping started. Drive the robot to build the map.", false);
    await loadOperatorPanel({ silent: true });
  } catch (error) {
    setMessage(elements.mappingMessage, error.message || "Mapping failed to start.", true);
  }
}

async function handleSaveMapping() {
  const mapName = elements.mappingMapName.value.trim();
  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.mappingMessage, "Select a robot before saving a map.", true);
    return;
  }
  if (!mapName) {
    setMessage(elements.mappingMessage, "Enter a map name before saving.", true);
    return;
  }

  elements.saveMappingButton.disabled = true;
  try {
    const response = await fetch(`/robots/${encodeURIComponent(robot.id)}/maps/save`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        map_name: mapName,
        command_source: getCommandSource(),
      }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Map save failed.");
    }

    await sendSystemCommand("launch_nav", { mapName, messageTarget: elements.mappingMessage });
    elements.mappingMapName.value = "";
    setMessage(elements.startNextMessage, `Map saved and selected: ${mapName}`, false);
    await loadOperatorPanel({ silent: true });
    await loadDashboardMapPreview({ silent: true });
    showScreen("start");
  } catch (error) {
    setMessage(elements.mappingMessage, error.message || "Map save failed.", true);
  } finally {
    elements.saveMappingButton.disabled = false;
  }
}

function handleAddMapMode(event) {
  const button = event.target.closest("[data-map-mode]");
  if (!button) {
    return;
  }
  state.operatorPanel.addMapMode = button.dataset.mapMode || "move";
  elements.addMapModes.querySelectorAll("[data-map-mode]").forEach((modeButton) => {
    modeButton.classList.toggle("active", modeButton === button);
  });
}

function handleAddMapClick(event) {
  const mode = state.operatorPanel.addMapMode;
  if (mode === "move") {
    setMessage(elements.addDestinationMessage, "Choose Set Destination Point or Set Initial Pose Point before clicking the map.", false);
    return;
  }
  const world = canvasPointToWorld(elements.addMapCanvas, event);
  if (!world) {
    setMessage(elements.addDestinationMessage, "Click inside the live map area.", true);
    return;
  }

  if (mode === "destination") {
    elements.destinationX.value = world.x.toFixed(2);
    elements.destinationY.value = world.y.toFixed(2);
    if (!elements.destinationYaw.value) {
      elements.destinationYaw.value = "0";
    }
    setMessage(elements.addDestinationMessage, `Destination point set at x ${formatNumber(world.x)}, y ${formatNumber(world.y)}.`, false);
    return;
  }

  elements.initialPoseX.value = world.x.toFixed(2);
  elements.initialPoseY.value = world.y.toFixed(2);
  if (!elements.initialPoseYaw.value) {
    elements.initialPoseYaw.value = "0";
  }
  setMessage(elements.addDestinationMessage, `Initial pose point set at x ${formatNumber(world.x)}, y ${formatNumber(world.y)}.`, false);
}

function fillInitialPoseFromRobot() {
  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.addDestinationMessage, "Select a robot before using current pose.", true);
    return;
  }
  elements.initialPoseX.value = Number(robot.x ?? 0).toFixed(2);
  elements.initialPoseY.value = Number(robot.y ?? 0).toFixed(2);
  elements.initialPoseYaw.value = Number(robot.yaw ?? 0).toFixed(2);
  setMessage(elements.addDestinationMessage, "Initial pose fields filled from current robot pose.", false);
}

function fillCalibrationPoseFromRobot() {
  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.calibrationMessage, "Select a robot before using current pose.", true);
    return;
  }
  elements.calibrationPoseX.value = Number(robot.x ?? 0).toFixed(2);
  elements.calibrationPoseY.value = Number(robot.y ?? 0).toFixed(2);
  elements.calibrationPoseYaw.value = Number(robot.yaw ?? 0).toFixed(2);
  setMessage(elements.calibrationMessage, "Initial pose fields filled from current robot pose.", false);
}

async function handleSendInitialPose() {
  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.addDestinationMessage, "Select a robot before sending an initial pose.", true);
    return;
  }

  const x = Number(elements.initialPoseX.value);
  const y = Number(elements.initialPoseY.value);
  const yaw = Number(elements.initialPoseYaw.value || 0);
  if ([x, y, yaw].some((value) => Number.isNaN(value))) {
    setMessage(elements.addDestinationMessage, "Enter valid initial pose values before sending.", true);
    return;
  }

  elements.sendInitialPoseButton.disabled = true;
  try {
    await sendInitialPose(robot.id, x, y, yaw);
    setMessage(elements.addDestinationMessage, `Initial pose sent at x ${formatNumber(x)}, y ${formatNumber(y)}.`, false);
    await loadOperatorPanel({ silent: true });
  } catch (error) {
    setMessage(elements.addDestinationMessage, error.message || "Initial pose update failed.", true);
  } finally {
    elements.sendInitialPoseButton.disabled = false;
  }
}

async function handleSendCalibrationInitialPose() {
  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.calibrationMessage, "Select a robot before sending an initial pose.", true);
    return;
  }

  const x = Number(elements.calibrationPoseX.value);
  const y = Number(elements.calibrationPoseY.value);
  const yaw = Number(elements.calibrationPoseYaw.value || 0);
  if ([x, y, yaw].some((value) => Number.isNaN(value))) {
    setMessage(elements.calibrationMessage, "Enter valid initial pose values before sending.", true);
    return;
  }

  elements.calibrationSendInitialPoseButton.disabled = true;
  try {
    await sendInitialPose(robot.id, x, y, yaw);
    setMessage(elements.calibrationMessage, `Initial pose sent at x ${formatNumber(x)}, y ${formatNumber(y)}.`, false);
    await loadOperatorPanel({ silent: true });
  } catch (error) {
    setMessage(elements.calibrationMessage, error.message || "Initial pose update failed.", true);
  } finally {
    elements.calibrationSendInitialPoseButton.disabled = false;
  }
}

async function handleDashboardMapClick(event) {
  if (state.activeScreen !== "start" || state.pointMissionInFlight) {
    return;
  }
  if (event.button !== 0 || state.operatorPanel.mapView.pointerMoved) {
    state.operatorPanel.mapView.pointerMoved = false;
    return;
  }

  const world = canvasPointToWorld(elements.stateMapCanvas, event);
  if (!world) {
    setMessage(elements.mapMessage, "Click inside the map area.", true);
    return;
  }

  const robot = getSelectedRobot();
  const x = Number(world.x.toFixed(2));
  const y = Number(world.y.toFixed(2));
  const yaw = Number(robot?.yaw || 0);
  promptMapGoal({
    name: "",
    pose: { x, y, yaw },
    configuredDestination: false,
  });
}

function promptMapGoal(goal) {
  state.operatorPanel.pendingGoal = {
    name: goal.name || "",
    configuredDestination: Boolean(goal.configuredDestination),
    pose: {
      x: Number(Number(goal.pose.x).toFixed(2)),
      y: Number(Number(goal.pose.y).toFixed(2)),
      yaw: Number(Number(goal.pose.yaw || 0).toFixed(2)),
    },
  };
  setMessage(elements.mapMessage, "", false);
  renderMap("state");
}

function clearPendingMapGoal() {
  state.operatorPanel.pendingGoal = null;
  if (elements.mapConfirmPopover) {
    elements.mapConfirmPopover.classList.add("hidden");
  }
  renderMap("state");
}

function renderMapConfirmPopover() {
  const pending = state.operatorPanel.pendingGoal;
  const popover = elements.mapConfirmPopover;
  if (!popover || !pending) {
    if (popover) {
      popover.classList.add("hidden");
    }
    return;
  }

  const frame = state.operatorPanel.frames[elements.stateMapCanvas.id];
  if (!frame) {
    popover.classList.add("hidden");
    return;
  }

  const point = worldToCanvasWithFrame(pending.pose, frame);
  const left = (point.x / elements.stateMapCanvas.width) * 100;
  const top = (point.y / elements.stateMapCanvas.height) * 100;
  elements.mapConfirmText.textContent = pending.name
    ? `Are you sure you want to go to ${pending.name}?`
    : "Are you sure you want to go to this point?";
  popover.style.left = `${Math.max(2, Math.min(88, left)).toFixed(3)}%`;
  popover.style.top = `${Math.max(8, Math.min(86, top)).toFixed(3)}%`;
  popover.classList.remove("hidden");
}

async function executePendingMapGoal() {
  if (!state.operatorPanel.pendingGoal || state.pointMissionInFlight) {
    return;
  }

  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.mapMessage, "Select a robot first.", true);
    return;
  }

  const pending = state.operatorPanel.pendingGoal;
  const destinationLabel = pending.name || `x ${formatNumber(pending.pose.x)}, y ${formatNumber(pending.pose.y)}`;
  state.pointMissionInFlight = true;
  elements.confirmMapGo.disabled = true;
  setMessage(elements.mapMessage, `Sending ${robot.id} to ${destinationLabel}.`, false);

  try {
    await sendDashboardGoal(robot, pending);
    state.operatorPanel.pendingGoal = null;
    await Promise.all([loadDestinations(), loadSnapshot(), loadOperatorPanel({ silent: true })]);
    setMessage(elements.mapMessage, `Destination set to ${destinationLabel}.`, false);
  } catch (error) {
    setMessage(elements.mapMessage, error.message || "Point mission failed.", true);
    await Promise.all([loadSnapshot(), loadOperatorPanel({ silent: true })]);
  } finally {
    state.pointMissionInFlight = false;
    elements.confirmMapGo.disabled = false;
    renderMap("state");
  }
}

async function sendDashboardGoal(robot, pending) {
  const { x, y, yaw } = pending.pose;
  await sendGoalPose(robot.id, x, y, yaw);

  const activeMission = getActiveMission();
  if (activeMission) {
    await upsertTempMapDestination(pending);
    return;
  }

  let toDestination = "Temp Destination";
  if (pending.configuredDestination && state.destinations.some((destination) => destination.name === pending.name)) {
    toDestination = pending.name;
  } else {
    await upsertTempMapDestination(pending);
  }

  const requestResponse = await fetch("/requests", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      requested_by: "map-click",
      command_source: getCommandSource(),
      to_destination: toDestination,
      schedule_type: "single",
      assigned_robot_id: robot.id,
      notes: pending.name
        ? `Named destination ${pending.name} on ${dashboardMapDisplayName()}.`
        : `Point selected at x ${formatNumber(x)}, y ${formatNumber(y)}.`,
    }),
  });
  const requestBody = await requestResponse.json();
  if (!requestResponse.ok) {
    throw new Error(requestBody.detail || "Point request creation failed.");
  }

  const startResponse = await fetch(`/requests/${encodeURIComponent(requestBody.request_id)}/start`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ command_source: getCommandSource() }),
  });
  const startBody = await startResponse.json();
  if (!startResponse.ok) {
    throw new Error(startBody.detail || "Point mission start failed.");
  }
}

async function upsertTempMapDestination(pending) {
  const { x, y, yaw } = pending.pose;
  const response = await fetch("/destinations/temp", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      x,
      y,
      yaw,
      notes: pending.name
        ? `Selected ${pending.name} on ${dashboardMapDisplayName()}.`
        : `Map click on ${dashboardMapDisplayName()}.`,
      command_source: getCommandSource(),
    }),
  });
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body.detail || "Point destination save failed.");
  }
}

async function handleSaveDestination() {
  const name = elements.destinationName.value.trim();
  const x = Number(elements.destinationX.value);
  const y = Number(elements.destinationY.value);
  const yaw = Number(elements.destinationYaw.value || 0);
  const robot = getSelectedRobot();
  if (!name) {
    setMessage(elements.addDestinationMessage, "Enter a destination name.", true);
    return;
  }
  if ([x, y, yaw].some((value) => Number.isNaN(value))) {
    setMessage(elements.addDestinationMessage, "Enter valid destination x, y, and yaw values.", true);
    return;
  }

  elements.saveDestinationButton.disabled = true;
  try {
    if (robot) {
      await sendGoalPose(robot.id, x, y, yaw);
      const initialValues = [elements.initialPoseX.value, elements.initialPoseY.value, elements.initialPoseYaw.value];
      if (initialValues.some((value) => value.trim() !== "")) {
        const ix = Number(elements.initialPoseX.value);
        const iy = Number(elements.initialPoseY.value);
        const iyaw = Number(elements.initialPoseYaw.value || 0);
        if ([ix, iy, iyaw].some((value) => Number.isNaN(value))) {
          throw new Error("Enter valid initial pose values or leave them blank.");
        }
        await sendInitialPose(robot.id, ix, iy, iyaw);
      }
    }

    const response = await fetch("/destinations", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        name,
        x,
        y,
        yaw,
        notes: `Saved from ${currentMapName() || "map"}`,
        command_source: getCommandSource(),
      }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Destination save failed.");
    }

    elements.destinationName.value = "";
    elements.destinationX.value = "";
    elements.destinationY.value = "";
    elements.destinationYaw.value = "0";
    await loadDestinations();
    setMessage(elements.startNextMessage, `Destination saved: ${name}`, false);
    showScreen("start");
  } catch (error) {
    setMessage(elements.addDestinationMessage, error.message || "Destination save failed.", true);
  } finally {
    elements.saveDestinationButton.disabled = false;
  }
}

function syncTripType() {
  const isRoundTrip = elements.tripType.value === "round_trip";
  elements.returnDestinationField.hidden = !isRoundTrip;
  elements.returnDestination.disabled = !isRoundTrip;
}

async function handleCreateRequest(event) {
  event.preventDefault();
  const payload = {
    requested_by: elements.requester.value.trim(),
    command_source: getCommandSource(),
    to_destination: elements.requestDestination.value,
    schedule_type: elements.tripType.value,
    notes: elements.requestNotes.value.trim(),
  };
  if (payload.schedule_type === "round_trip" && elements.returnDestination.value) {
    payload.from_destination = elements.returnDestination.value;
  }
  if (elements.requestRobot.value) {
    payload.assigned_robot_id = elements.requestRobot.value;
  }

  elements.createRequestButton.disabled = true;
  try {
    const response = await fetch("/requests", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Request creation failed.");
    }
    setMessage(elements.requestMessage, `Request #${String(body.request_number).padStart(3, "0")} created.`, false);
    elements.createAnotherButton.classList.remove("hidden");
    elements.goStateButton.classList.remove("hidden");
    await loadSnapshot();
  } catch (error) {
    setMessage(elements.requestMessage, error.message || "Request creation failed.", true);
  } finally {
    elements.createRequestButton.disabled = false;
  }
}

function resetRequestForm() {
  elements.requestNotes.value = "";
  elements.createAnotherButton.classList.add("hidden");
  elements.goStateButton.classList.add("hidden");
  setMessage(elements.requestMessage, "", false);
}

async function handleStartRequestClick(event) {
  const button = event.target.closest("[data-start-request]");
  if (!button) {
    return;
  }
  const requestId = button.dataset.startRequest;
  button.disabled = true;
  try {
    const response = await fetch(`/requests/${encodeURIComponent(requestId)}/start`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command_source: getCommandSource() }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Mission start failed.");
    }
    setMessage(elements.stateMessage, `Mission started from Request #${String(body.request_number).padStart(3, "0")}.`, false);
    await Promise.all([loadSnapshot(), loadOperatorPanel({ silent: true })]);
  } catch (error) {
    setMessage(elements.stateMessage, error.message || "Mission start failed.", true);
  } finally {
    button.disabled = false;
  }
}

async function handleCancelCurrentMission(event) {
  const button = event.currentTarget;
  const activeMission = state.missions.find((item) =>
    item.state !== "Requested" &&
    item.state !== "Completed" &&
    item.outcome !== "Canceled" &&
    item.outcome !== "Failed" &&
    item.outcome !== "Aborted"
  );

  if (activeMission) {
    button.dataset.action = "cancel";
    button.dataset.missionId = activeMission.id;
    await handleMissionAction({ target: button });
    return;
  }

  setMessage(elements.stateMessage, "No active mission to cancel.", false);
}

async function handleMissionAction(event) {
  const button = event.target.closest("[data-action]");
  if (!button) {
    return;
  }
  const action = button.dataset.action;
  const missionId = button.dataset.missionId;
  if (!missionId) {
    return;
  }
  button.disabled = true;
  try {
    const response = await fetch(`/missions/${encodeURIComponent(missionId)}/${action}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command_source: getCommandSource() }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || `${action} failed.`);
    }
    if (action === "return") {
      state.returnPromptDismissed.add(missionId);
      hideReturnModal();
      setMessage(elements.stateMessage, `Return trip started for ${formatRequestNumberById(missionId)}.`, false);
    } else {
      setMessage(elements.stateMessage, `${displayMissionAction(action)} sent.`, false);
    }
    await loadSnapshot();
  } catch (error) {
    setMessage(elements.stateMessage, error.message || `${action} failed.`, true);
  } finally {
    button.disabled = false;
  }
}

async function handlePowerAction(event) {
  const button = event.target.closest("[data-power-mode]");
  if (!button) {
    return;
  }
  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.manualMessage, "Select a robot first.", true);
    return;
  }
  stopManualDrive({ silent: true });
  button.disabled = true;
  try {
    const response = await fetch(`/robots/${encodeURIComponent(robot.id)}/power/set-mode`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        mode: button.dataset.powerMode,
        command_source: getCommandSource(),
      }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Robot control failed.");
    }
    setMessage(elements.manualMessage, `${displayPowerMode(button.dataset.powerMode)} sent.`, false);
    await loadSnapshot();
  } catch (error) {
    setMessage(elements.manualMessage, error.message || "Robot control failed.", true);
  } finally {
    button.disabled = false;
  }
}

function handleManualPadPointerDown(event) {
  const button = event.target.closest("[data-manual-linear], [data-manual-stop]");
  if (!button || button.disabled) {
    return;
  }
  event.preventDefault();
  if (button.dataset.manualStop === "true") {
    stopManualDrive({ sendStop: false, silent: true });
    button.classList.add("is-active");
    window.setTimeout(() => button.classList.remove("is-active"), 180);
    queueManualDriveCommand(0, 0);
    setMessage(elements.manualMessage, "Stop Movement sent.", false);
    return;
  }

  startManualDrive(
    Number(button.dataset.manualLinear),
    Number(button.dataset.manualAngular),
    button.dataset.manualLabel || button.textContent.trim(),
    button
  );
}

async function handleSaveTempDestination() {
  const goalPose = state.operatorPanel.data?.goal_pose;
  if (!goalPose) {
    setMessage(elements.mapMessage, "No goal position is available to save.", true);
    return;
  }
  try {
    const response = await fetch("/destinations/temp", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        x: goalPose.x,
        y: goalPose.y,
        yaw: goalPose.yaw || 0,
        notes: "Saved from State map.",
        command_source: getCommandSource(),
      }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Temp destination save failed.");
    }
    await loadDestinations();
    setMessage(elements.mapMessage, "Goal saved as Temp Destination.", false);
  } catch (error) {
    setMessage(elements.mapMessage, error.message || "Temp destination save failed.", true);
  }
}

async function handleQueueReset({ button, endpoint, confirmText, successLabel, messageTarget = elements.queueMessage }) {
  if (!window.confirm(confirmText)) {
    return;
  }
  button.disabled = true;
  try {
    const response = await fetch(endpoint, { method: "POST" });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Queue update failed.");
    }
    setMessage(messageTarget, `${successLabel}: ${body.deleted_missions} removed.`, false);
    await loadSnapshot();
  } catch (error) {
    setMessage(messageTarget, error.message || "Queue update failed.", true);
  } finally {
    button.disabled = false;
  }
}

async function sendSystemCommand(command, { mapName = null, messageTarget = elements.manageMessage } = {}) {
  const robot = getSelectedRobot();
  if (!robot) {
    throw new Error("Select a robot first.");
  }
  const response = await fetch(`/robots/${encodeURIComponent(robot.id)}/system-command`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      command,
      map_name: mapName,
      command_source: getCommandSource(),
    }),
  });
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body.detail || "Robot command failed.");
  }
  setMessage(messageTarget, `${displaySystemCommand(command)} sent.`, false);
  return body;
}

async function sendGoalPose(robotId, x, y, yaw) {
  const response = await fetch(`/robots/${encodeURIComponent(robotId)}/goal-pose`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ x, y, yaw, command_source: getCommandSource() }),
  });
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body.detail || "Goal position update failed.");
  }
  if (state.operatorPanel.data) {
    state.operatorPanel.data.goal_pose = { x, y, yaw };
  }
}

async function sendInitialPose(robotId, x, y, yaw) {
  const response = await fetch(`/robots/${encodeURIComponent(robotId)}/initial-pose`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ x, y, yaw, command_source: getCommandSource() }),
  });
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body.detail || "Initial position update failed.");
  }
  if (state.operatorPanel.data) {
    state.operatorPanel.data.initial_pose = { x, y, yaw };
  }
}

function buildMissionActionButton(action, mission) {
  if (mission.state === "Completed") {
    return "";
  }
  if (action === "pause" && !["En-route", "Returning"].includes(mission.state)) {
    return "";
  }
  if (action === "resume" && mission.state !== "Paused") {
    return "";
  }
  if (action === "return" && mission.state !== "WaitingForReturn") {
    return "";
  }
  return `<button class="action-button ${action}" type="button" data-action="${action}" data-mission-id="${escapeHtml(mission.id)}">${displayMissionAction(action)}</button>`;
}

function startManualDrive(linearDir, angularDir, label, button) {
  const robot = getSelectedRobot();
  if (!robot) {
    setMessage(elements.manualMessage, "Select a robot before driving manually.", true);
    return;
  }
  if (!isManualDriveAvailable(robot)) {
    setMessage(elements.manualMessage, "Turn the robot on before driving manually.", true);
    return;
  }

  stopManualDrive({ sendStop: false, silent: true });
  state.manualDrive.currentLinear = linearDir * MANUAL_BASE_SPEED;
  state.manualDrive.currentAngular = angularDir * MANUAL_BASE_SPEED;
  state.manualDrive.activeButton = button;
  button.classList.add("is-active");
  queueManualDriveCommand(state.manualDrive.currentLinear, state.manualDrive.currentAngular);
  setMessage(elements.manualMessage, `Manual drive: ${label}`, false);

  state.manualDrive.activeTimer = window.setInterval(() => {
    if (Math.abs(state.manualDrive.currentLinear) < MANUAL_MAX_SPEED) {
      state.manualDrive.currentLinear += linearDir * MANUAL_ACCEL_RATE;
      if (Math.abs(state.manualDrive.currentLinear) > MANUAL_MAX_SPEED) {
        state.manualDrive.currentLinear = linearDir * MANUAL_MAX_SPEED;
      }
    }
    if (Math.abs(state.manualDrive.currentAngular) < MANUAL_MAX_SPEED) {
      state.manualDrive.currentAngular += angularDir * MANUAL_ACCEL_RATE;
      if (Math.abs(state.manualDrive.currentAngular) > MANUAL_MAX_SPEED) {
        state.manualDrive.currentAngular = angularDir * MANUAL_MAX_SPEED;
      }
    }
    queueManualDriveCommand(state.manualDrive.currentLinear, state.manualDrive.currentAngular);
  }, MANUAL_TICK_MS);
}

function stopManualDrive({ sendStop = true, silent = false } = {}) {
  const hadCommand =
    state.manualDrive.activeTimer !== null ||
    Math.abs(state.manualDrive.currentLinear) > 1e-4 ||
    Math.abs(state.manualDrive.currentAngular) > 1e-4;

  if (state.manualDrive.activeTimer !== null) {
    window.clearInterval(state.manualDrive.activeTimer);
    state.manualDrive.activeTimer = null;
  }
  if (state.manualDrive.activeButton) {
    state.manualDrive.activeButton.classList.remove("is-active");
    state.manualDrive.activeButton = null;
  }
  state.manualDrive.currentLinear = 0;
  state.manualDrive.currentAngular = 0;

  if (sendStop && hadCommand) {
    queueManualDriveCommand(0, 0);
  }
  if (hadCommand && !silent) {
    setMessage(elements.manualMessage, "Manual drive stopped.", false);
  }
}

function queueManualDriveCommand(linear, angular) {
  state.manualDrive.pendingCommand = {
    linear: Number(linear.toFixed(3)),
    angular: Number(angular.toFixed(3)),
  };
  if (!state.manualDrive.requestInFlight) {
    void flushManualDriveCommand();
  }
}

async function flushManualDriveCommand() {
  const nextCommand = state.manualDrive.pendingCommand;
  if (!nextCommand) {
    return;
  }
  const robot = getSelectedRobot();
  if (!robot) {
    state.manualDrive.pendingCommand = null;
    return;
  }

  state.manualDrive.pendingCommand = null;
  state.manualDrive.requestInFlight = true;
  try {
    const response = await fetch(`/robots/${encodeURIComponent(robot.id)}/manual-drive`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        linear: nextCommand.linear,
        angular: nextCommand.angular,
        command_source: getCommandSource(),
      }),
    });
    const body = await response.json();
    if (!response.ok) {
      throw new Error(body.detail || "Manual drive failed.");
    }
  } catch (error) {
    setMessage(elements.manualMessage, error.message || "Manual drive failed.", true);
    stopManualDrive({ sendStop: false, silent: true });
  } finally {
    state.manualDrive.requestInFlight = false;
    if (state.manualDrive.pendingCommand) {
      void flushManualDriveCommand();
    }
  }
}

function drawMap(canvas, mapData, markers) {
  const ctx = canvas.getContext("2d");
  clearCanvas(canvas);
  const raster = getRenderedMapRaster(mapData);
  const padding = 16;
  const view = markers.view || {};
  const baseScale = Math.min((canvas.width - padding * 2) / mapData.width, (canvas.height - padding * 2) / mapData.height);
  const scale = baseScale * (Number(view.zoom) || 1);
  const drawWidth = mapData.width * scale;
  const drawHeight = mapData.height * scale;
  const baseOffsetX = (canvas.width - drawWidth) / 2;
  const baseOffsetY = (canvas.height - drawHeight) / 2;
  if (view.followRobot && markers.robot) {
    centerMapViewOnPose(view, markers.robot, mapData, canvas, baseOffsetX, baseOffsetY, scale);
  }
  const offsetX = baseOffsetX + (Number(view.panX) || 0);
  const offsetY = baseOffsetY + (Number(view.panY) || 0);
  const frame = { map: mapData, offsetX, offsetY, drawWidth, drawHeight, scale };

  ctx.drawImage(raster, offsetX, offsetY, drawWidth, drawHeight);
  ctx.strokeStyle = "rgba(23, 39, 36, 0.24)";
  ctx.strokeRect(offsetX, offsetY, drawWidth, drawHeight);

  if (markers.initialPose) {
    drawInitialPoseMarker(ctx, worldToCanvasWithFrame(markers.initialPose, frame), markers.initialPose.yaw || 0);
  }
  if (markers.goalPose) {
    drawGoalMarker(ctx, worldToCanvasWithFrame(markers.goalPose, frame));
  }
  if (markers.pendingGoal) {
    drawPendingGoalMarker(ctx, worldToCanvasWithFrame(markers.pendingGoal, frame));
  }
  if (markers.robot) {
    drawRobotMarker(ctx, worldToCanvasWithFrame(markers.robot, frame), markers.robot.yaw || 0, frame);
  }

  state.operatorPanel.frames[canvas.id] = frame;
}

function getRenderedMapRaster(mapData) {
  const key = `${mapData.name || "live"}:${mapData.width}:${mapData.height}:${mapData.updated_at}`;
  if (state.operatorPanel.renderedMapKey === key && state.operatorPanel.renderedMapCanvas) {
    return state.operatorPanel.renderedMapCanvas;
  }

  const canvas = document.createElement("canvas");
  canvas.width = mapData.width;
  canvas.height = mapData.height;
  const ctx = canvas.getContext("2d");
  const imageData = ctx.createImageData(mapData.width, mapData.height);
  for (let row = 0; row < mapData.height; row += 1) {
    for (let col = 0; col < mapData.width; col += 1) {
      const sourceIndex = row * mapData.width + col;
      const value = mapData.data[sourceIndex];
      const targetRow = mapData.height - 1 - row;
      const pixelIndex = (targetRow * mapData.width + col) * 4;
      const shade = value < 0 ? 214 : 255 - Math.round((Math.min(100, Math.max(0, value)) / 100) * 255);
      imageData.data[pixelIndex] = shade;
      imageData.data[pixelIndex + 1] = shade;
      imageData.data[pixelIndex + 2] = shade;
      imageData.data[pixelIndex + 3] = 255;
    }
  }
  ctx.putImageData(imageData, 0, 0);
  state.operatorPanel.renderedMapKey = key;
  state.operatorPanel.renderedMapCanvas = canvas;
  return canvas;
}

function centerMapViewOnPose(view, pose, mapData, canvas, baseOffsetX, baseOffsetY, scale) {
  const point = worldToCanvas(pose, mapData, baseOffsetX, baseOffsetY, scale);
  if ([point.x, point.y].some((value) => !Number.isFinite(value))) {
    return;
  }
  view.panX = (canvas.width / 2) - point.x;
  view.panY = (canvas.height / 2) - point.y;
}

function canvasPointToWorld(canvas, event) {
  const frame = state.operatorPanel.frames[canvas.id];
  if (!frame) {
    return null;
  }
  const rect = canvas.getBoundingClientRect();
  const canvasX = (event.clientX - rect.left) * (canvas.width / rect.width);
  const canvasY = (event.clientY - rect.top) * (canvas.height / rect.height);
  if (
    canvasX < frame.offsetX ||
    canvasX > frame.offsetX + frame.drawWidth ||
    canvasY < frame.offsetY ||
    canvasY > frame.offsetY + frame.drawHeight
  ) {
    return null;
  }
  const gridX = (canvasX - frame.offsetX) / frame.scale;
  const gridY = frame.map.height - ((canvasY - frame.offsetY) / frame.scale);
  return {
    x: frame.map.origin.x + gridX * frame.map.resolution,
    y: frame.map.origin.y + gridY * frame.map.resolution,
  };
}

function worldToCanvas(pose, mapData, offsetX, offsetY, scale) {
  const gridX = (Number(pose.x) - mapData.origin.x) / mapData.resolution;
  const gridY = (Number(pose.y) - mapData.origin.y) / mapData.resolution;
  return {
    x: offsetX + gridX * scale,
    y: offsetY + (mapData.height - gridY) * scale,
  };
}

function worldToCanvasWithFrame(pose, frame) {
  return worldToCanvas(pose, frame.map, frame.offsetX, frame.offsetY, frame.scale);
}

function isPointWithinFrame(point, frame) {
  return (
    point.x >= frame.offsetX &&
    point.x <= frame.offsetX + frame.drawWidth &&
    point.y >= frame.offsetY &&
    point.y <= frame.offsetY + frame.drawHeight
  );
}

function drawRobotMarker(ctx, point, yaw, frame) {
  const pixelsPerMeter = frame.scale / frame.map.resolution;
  const front = ROBOT_FOOTPRINT_FRONT_M * pixelsPerMeter;
  const rear = ROBOT_FOOTPRINT_REAR_M * pixelsPerMeter;
  const width = ROBOT_FOOTPRINT_WIDTH_M * pixelsPerMeter;
  const markerLength = front + rear;
  const strokeWidth = Math.max(2, Math.min(5, pixelsPerMeter * 0.035));

  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-(Number(yaw) || 0));
  ctx.fillStyle = "rgba(72, 92, 114, 0.92)";
  ctx.strokeStyle = "#f7f9fb";
  ctx.lineWidth = strokeWidth;

  ctx.beginPath();
  ctx.rect(-rear, -width / 2, markerLength, width);
  ctx.fill();
  ctx.stroke();

  ctx.strokeStyle = "#dbe4ec";
  ctx.lineWidth = Math.max(1.5, strokeWidth * 0.65);
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(front * 0.78, 0);
  ctx.moveTo(front * 0.62, -width * 0.18);
  ctx.lineTo(front * 0.82, 0);
  ctx.lineTo(front * 0.62, width * 0.18);
  ctx.stroke();

  ctx.fillStyle = "#f7f9fb";
  ctx.beginPath();
  ctx.arc(0, 0, Math.max(2, Math.min(5, pixelsPerMeter * 0.045)), 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawInitialPoseMarker(ctx, point, yaw) {
  ctx.save();
  ctx.strokeStyle = "#9a641c";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(point.x, point.y, 10, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(point.x, point.y);
  ctx.lineTo(point.x + Math.cos(-(Number(yaw) || 0)) * 15, point.y + Math.sin(-(Number(yaw) || 0)) * 15);
  ctx.stroke();
  ctx.restore();
}

function drawGoalMarker(ctx, point) {
  ctx.save();
  ctx.strokeStyle = "#b33a32";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(point.x - 8, point.y - 8);
  ctx.lineTo(point.x + 8, point.y + 8);
  ctx.moveTo(point.x + 8, point.y - 8);
  ctx.lineTo(point.x - 8, point.y + 8);
  ctx.stroke();
  ctx.restore();
}

function drawPendingGoalMarker(ctx, point) {
  ctx.save();
  ctx.fillStyle = "rgba(18, 83, 134, 0.18)";
  ctx.strokeStyle = "#125386";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(point.x, point.y, 14, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(point.x - 18, point.y);
  ctx.lineTo(point.x + 18, point.y);
  ctx.moveTo(point.x, point.y - 18);
  ctx.lineTo(point.x, point.y + 18);
  ctx.stroke();
  ctx.restore();
}

function clearCanvas(canvas) {
  if (!canvas) {
    return;
  }
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#f7faf9";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  delete state.operatorPanel.frames[canvas.id];
}

function getSelectedRobot() {
  return state.robots.find((robot) => robot.id === elements.selectedRobot.value) || null;
}

function getMapContextRobot() {
  return getSelectedRobot() || sortedRobotsForSelection()[0] || null;
}

function isRobotVisibleOnMap(robot) {
  return Boolean(robot && robot.online !== false && robot.connection_ok && robot.localization_valid);
}

function sortedRobotsForSelection() {
  return [...state.robots].sort((a, b) => {
    const aConnected = robotConnectionLabel(a) === "Connected" ? 0 : 1;
    const bConnected = robotConnectionLabel(b) === "Connected" ? 0 : 1;
    if (aConnected !== bConnected) {
      return aConnected - bConnected;
    }
    const aOnline = a.online === false ? 1 : 0;
    const bOnline = b.online === false ? 1 : 0;
    if (aOnline !== bOnline) {
      return aOnline - bOnline;
    }
    const aHasPower = a.power ? 0 : 1;
    const bHasPower = b.power ? 0 : 1;
    if (aHasPower !== bHasPower) {
      return aHasPower - bHasPower;
    }
    return String(a.id || "").localeCompare(String(b.id || ""));
  });
}

function getSavedMaps() {
  return state.operatorPanel.data?.saved_maps ?? [];
}

function currentMapName() {
  return state.operatorPanel.data?.current_map_name || "";
}

function dashboardMapDisplayName() {
  return state.operatorPanel.mapPreview?.name || currentMapName() || "";
}

function getPendingRequests() {
  return state.missions
    .filter((mission) => mission.state === "Requested")
    .sort((a, b) => Number(a.created_at || 0) - Number(b.created_at || 0));
}

function getActiveMission() {
  return state.missions.find((item) =>
    item.state !== "Requested" &&
    item.state !== "Completed" &&
    item.outcome !== "Canceled" &&
    item.outcome !== "Failed" &&
    item.outcome !== "Aborted"
  ) || null;
}

function getWaitingForReturnMission() {
  return state.missions
    .filter((mission) => mission.state === "WaitingForReturn")
    .sort((a, b) => Number(a.last_update_at || 0) - Number(b.last_update_at || 0))[0] || null;
}

function getCommandSource() {
  return {
    type: "operator",
    id: elements.operatorId.value.trim() || "dashboard-1",
  };
}

function robotConnectionLabel(robot) {
  if (!robot) {
    return "--";
  }
  return robot.online === false || !robot.connection_ok ? "Disconnected" : "Connected";
}

function robotWarningLabel(robot, battery) {
  const power = robot?.power ?? {};
  const warnings = [];
  const explicitStatus = formatExplicitStatus(robot?.error || robot?.warning || power.error || power.warning);
  if (explicitStatus) {
    warnings.push(explicitStatus);
  }
  if (robotConnectionLabel(robot) === "Disconnected") {
    warnings.push("Disconnected");
  }
  if (battery != null && Number(battery) < 20) {
    warnings.push("Low battery");
  }
  if (power.safety_lock) {
    warnings.push("Safety lock");
  }
  if (robot?.localization_valid === false) {
    warnings.push("Localization");
  }
  return warnings.length ? warnings.join(", ") : "None";
}

function formatExplicitStatus(value) {
  if (!value) {
    return "";
  }
  if (typeof value === "string") {
    return value;
  }
  if (Array.isArray(value)) {
    return value.filter(Boolean).join(", ");
  }
  return "Robot warning";
}

function isManualDriveAvailable(robot = getSelectedRobot()) {
  if (!robot) {
    return false;
  }
  const power = robot.power ?? {};
  const mode = power.mode || (robot.mode === "ManualOverride" ? "MANUAL" : "AUTO");
  return Boolean(power.available) && !["STOP", "OFF"].includes(mode) && !power.safety_lock;
}

function formatRoute(mission) {
  if (mission.schedule_type === "round_trip") {
    return `${mission.to_dest} -> ${mission.from_dest || state.home || "Home"}`;
  }
  return mission.to_dest;
}

function formatRequestNumber(mission) {
  const ordered = [...state.missions].sort((a, b) => Number(a.created_at || 0) - Number(b.created_at || 0));
  const index = ordered.findIndex((item) => item.id === mission.id);
  return `Request #${String(index + 1 || 1).padStart(3, "0")}`;
}

function formatRequestNumberById(missionId) {
  const mission = state.missions.find((item) => item.id === missionId);
  return mission ? formatRequestNumber(mission) : "mission";
}

function formatPoseShort(pose) {
  if (!pose) {
    return "--";
  }
  return `x ${formatNumber(pose.x)}, y ${formatNumber(pose.y)}, yaw ${formatNumber(pose.yaw || 0)}`;
}

function displayMissionStatus(mission) {
  if (mission.help_required) {
    return "Needs Help";
  }
  if (mission.outcome === "Canceled") {
    return "Canceled";
  }
  if (mission.outcome === "Failed" || mission.outcome === "Aborted") {
    return "Failed";
  }
  const labels = {
    Requested: "Pending Request",
    Idle: "Queued",
    "En-route": "In Progress",
    WaitingForReturn: "Waiting for Return",
    Returning: "Returning",
    Paused: "Paused",
    Completed: "Completed",
  };
  return labels[mission.state] || mission.state || "--";
}

function displayMissionAction(action) {
  const labels = {
    pause: "Pause",
    resume: "Resume",
    return: "Return",
    cancel: "Cancel Mission",
  };
  return labels[action] || action;
}

function displayPowerMode(mode) {
  const labels = {
    AUTO: "ON",
    MANUAL: "ON",
    RESET: "ON",
    STOP: "OFF",
    OFF: "OFF",
    ON: "ON",
  };
  return labels[String(mode || "").toUpperCase()] || String(mode || "--");
}

function displaySystemCommand(command) {
  const labels = {
    launch_slam: "Mapping Mode",
    launch_nav: "Map selected",
    launch_robot: "Robot System",
    kill_all: "Kill Launcher Processes",
  };
  return labels[command] || command;
}

function batteryPercentFromVoltage(voltage) {
  if (voltage == null || Number.isNaN(Number(voltage))) {
    return null;
  }
  const percent = ((Number(voltage) - 20.0) / 4.0) * 100.0;
  return Math.max(0, Math.min(100, percent));
}

function formatNumber(value) {
  if (value == null || Number.isNaN(Number(value))) {
    return "--";
  }
  return Number(value).toFixed(2);
}

function setText(element, value) {
  if (element) {
    element.textContent = value;
  }
}

function setMessage(element, message, isError) {
  if (!element) {
    return;
  }
  const existingTimer = messageTimers.get(element);
  if (existingTimer) {
    window.clearTimeout(existingTimer);
    messageTimers.delete(element);
  }

  element.textContent = message || "";
  element.classList.toggle("error", Boolean(message && isError));
  element.classList.toggle("success", Boolean(message && !isError));

  if (!message) {
    delete element.dataset.messageToken;
    return;
  }

  const token = `${Date.now()}:${Math.random()}`;
  element.dataset.messageToken = token;
  const timeout = isError ? MESSAGE_ERROR_TIMEOUT_MS : MESSAGE_SUCCESS_TIMEOUT_MS;
  const timer = window.setTimeout(() => {
    if (element.dataset.messageToken !== token) {
      return;
    }
    element.textContent = "";
    element.classList.remove("error", "success");
    delete element.dataset.messageToken;
    messageTimers.delete(element);
  }, timeout);
  messageTimers.set(element, timer);
}

function slugify(value) {
  return String(value || "")
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-");
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}
