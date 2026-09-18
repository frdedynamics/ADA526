(function () {
  const SVG_NS = "http://www.w3.org/2000/svg";

  function radians(degrees) {
    return (degrees * Math.PI) / 180;
  }

  function formatNumber(value) {
    const normalized = Math.abs(value) < 0.005 ? 0 : value;
    return normalized.toFixed(2);
  }

  function multiply3(left, right) {
    return left.map(function (row, rowIndex) {
      return right[0].map(function (_, columnIndex) {
        return row.reduce(function (sum, __, innerIndex) {
          return sum + left[rowIndex][innerIndex] * right[innerIndex][columnIndex];
        }, 0);
      });
    });
  }

  function multiply4(left, right) {
    return left.map(function (row, rowIndex) {
      return right[0].map(function (_, columnIndex) {
        return row.reduce(function (sum, __, innerIndex) {
          return sum + left[rowIndex][innerIndex] * right[innerIndex][columnIndex];
        }, 0);
      });
    });
  }

  function multiplyVector(matrix, vector) {
    return matrix.map(function (row) {
      return row.reduce(function (sum, value, index) {
        return sum + value * vector[index];
      }, 0);
    });
  }

  function addVectors(left, right) {
    return left.map(function (value, index) {
      return value + right[index];
    });
  }

  function scaleVector(vector, scale) {
    return vector.map(function (value) {
      return value * scale;
    });
  }

  function rotationZ(angle) {
    const cosine = Math.cos(angle);
    const sine = Math.sin(angle);
    return [
      [cosine, -sine, 0],
      [sine, cosine, 0],
      [0, 0, 1]
    ];
  }

  function rotationX(angle) {
    const cosine = Math.cos(angle);
    const sine = Math.sin(angle);
    return [
      [1, 0, 0],
      [0, cosine, -sine],
      [0, sine, cosine]
    ];
  }

  function matrixColumn(matrix, column) {
    return matrix.map(function (row) {
      return row[column];
    });
  }

  function lineTrace(points, color, width, dash) {
    return {
      type: "scatter3d",
      mode: "lines",
      x: points.map(function (point) { return point[0]; }),
      y: points.map(function (point) { return point[1]; }),
      z: points.map(function (point) { return point[2]; }),
      line: { color: color, width: width, dash: dash || "solid" },
      hoverinfo: "skip",
      showlegend: false
    };
  }

  function axisTrace(origin, direction, length, color, label, opacity) {
    const endpoint = addVectors(origin, scaleVector(direction, length));
    return {
      type: "scatter3d",
      mode: "lines+text",
      x: [origin[0], endpoint[0]],
      y: [origin[1], endpoint[1]],
      z: [origin[2], endpoint[2]],
      text: ["", label],
      textposition: "top center",
      textfont: { color: color, size: 12 },
      line: { color: color, width: 7 },
      opacity: opacity,
      hoverinfo: "skip",
      showlegend: false
    };
  }

  function jointAxisTrace(origin, direction, color, label, opacity) {
    const start = addVectors(origin, scaleVector(direction, -0.75));
    const end = addVectors(origin, scaleVector(direction, 0.75));
    return {
      type: "scatter3d",
      mode: "lines+text",
      x: [start[0], end[0]],
      y: [start[1], end[1]],
      z: [start[2], end[2]],
      text: ["", label],
      textposition: "top center",
      textfont: { color: color, size: 12 },
      line: { color: color, width: 4, dash: "dash" },
      opacity: opacity,
      hoverinfo: "skip",
      showlegend: false
    };
  }

  function pointLabelTrace(position, label, color) {
    return {
      type: "scatter3d",
      mode: "markers+text",
      x: [position[0]],
      y: [position[1]],
      z: [position[2]],
      text: [label],
      textposition: "bottom center",
      textfont: { color: color, size: 13 },
      marker: { color: color, size: 4 },
      hoverinfo: "skip",
      showlegend: false
    };
  }

  function addFrame(traces, origin, rotation, suffix, opacity) {
    const axisLength = 0.48;
    traces.push(axisTrace(origin, matrixColumn(rotation, 0), axisLength, "#c2413b", "x" + suffix, opacity));
    traces.push(axisTrace(origin, matrixColumn(rotation, 1), axisLength, "#16865c", "y" + suffix, opacity));
    traces.push(axisTrace(origin, matrixColumn(rotation, 2), axisLength, "#2563a6", "z" + suffix, opacity));
  }

  function initializeDhDemo(root) {
    const plotRoot = root.querySelector("[data-dh-plot]");
    const description = root.querySelector("[data-dh-description]");
    const buttons = Array.from(root.querySelectorAll("[data-dh-step]"));
    const inputs = Array.from(root.querySelectorAll("[data-dh-param]"));
    const linkTabs = root.querySelector("[data-dh-link-tabs]");
    const summaryBody = root.querySelector("[data-dh-summary]");
    const addButton = root.querySelector("[data-dh-add]");
    const removeButton = root.querySelector("[data-dh-remove]");
    const resetButton = root.querySelector("[data-dh-reset]");
    const maximumLinks = 6;
    const parameterSteps = { theta: 1, d: 2, a: 3, alpha: 4 };
    const presets = [
      { theta: 35, d: 0.65, a: 1.10, alpha: 55 },
      { theta: -30, d: 0.20, a: 0.90, alpha: -40 },
      { theta: 45, d: 0.15, a: 0.80, alpha: 35 },
      { theta: 20, d: 0.10, a: 0.70, alpha: -30 },
      { theta: -25, d: 0.10, a: 0.60, alpha: 30 },
      { theta: 30, d: 0.10, a: 0.50, alpha: -25 }
    ];
    let links = [createLink(0)];
    let activeLinkIndex = 0;
    let activeStep = 4;
    let camera = {
      eye: { x: 1.45, y: 1.55, z: 1.15 },
      up: { x: 0, y: 0, z: 1 }
    };

    function copyCamera(value) {
      return JSON.parse(JSON.stringify(value));
    }

    function rememberCamera() {
      if (plotRoot._fullLayout && plotRoot._fullLayout.scene && plotRoot._fullLayout.scene.camera) {
        camera = copyCamera(plotRoot._fullLayout.scene.camera);
      }
    }

    function createLink(index) {
      return Object.assign({}, presets[index]);
    }

    function currentLink() {
      return links[activeLinkIndex];
    }

    function stepDescription(step) {
      const linkNumber = activeLinkIndex + 1;
      const startFrame = activeLinkIndex;
      const subscripts = ["\u2080", "\u2081", "\u2082", "\u2083", "\u2084", "\u2085", "\u2086"];
      const theta = "\u03b8" + subscripts[linkNumber];
      const alpha = "\u03b1" + subscripts[linkNumber];
      const d = "d" + subscripts[linkNumber];
      const a = "a" + subscripts[linkNumber];
      const startZ = "z" + subscripts[startFrame];
      const endX = "x" + subscripts[linkNumber];
      const endZ = "z" + subscripts[linkNumber];
      const messages = [
        "Link " + linkNumber + ": start with frame {" + startFrame + "}. Its z-axis is the motion axis of joint " + linkNumber + ".",
        "Rotate by " + theta + " around " + startZ + ". The origin and joint axis stay in place while x and y rotate.",
        "Translate by " + d + " along " + startZ + ". This moves the frame origin along the joint axis.",
        "Translate by " + a + " along the rotated x direction. This common normal takes us to the next joint axis.",
        "Rotate by " + alpha + " around " + endX + ". This sets the direction of " + endZ + " and completes frame {" + linkNumber + "}."
      ];
      return messages[step];
    }

    function updateControls() {
      const link = currentLink();
      inputs.forEach(function (input) {
        const name = input.dataset.dhParam;
        input.value = link[name];
        input.disabled = parameterSteps[name] > activeStep;
        input.closest(".kinematics-slider").classList.toggle("is-disabled", input.disabled);
      });
      ["theta", "alpha"].forEach(function (name) {
        root.querySelector('[data-dh-output="' + name + '"]').textContent = link[name].toFixed(0) + "\u00b0";
      });
      ["d", "a"].forEach(function (name) {
        root.querySelector('[data-dh-output="' + name + '"]').textContent = link[name].toFixed(2);
      });

      buttons.forEach(function (button) {
        const step = Number(button.dataset.dhStep);
        button.disabled = false;
        button.classList.toggle("is-active", step === activeStep);
      });
    }

    function updateLinkTabs() {
      linkTabs.replaceChildren();
      links.forEach(function (link, index) {
        const button = document.createElement("button");
        button.type = "button";
        button.setAttribute("role", "tab");
        button.setAttribute("aria-selected", String(index === activeLinkIndex));
        button.textContent = "Link " + (index + 1);
        button.classList.toggle("is-active", index === activeLinkIndex);
        button.addEventListener("click", function () {
          activeLinkIndex = index;
          activeStep = 4;
          render();
        });
        linkTabs.appendChild(button);
      });
    }

    function updateActions() {
      addButton.disabled = activeLinkIndex !== links.length - 1 ||
        links.length >= maximumLinks;
      addButton.title = links.length >= maximumLinks ? "The demonstration supports up to six links" : "";
      removeButton.disabled = links.length === 1;
    }

    function updateSummary() {
      summaryBody.replaceChildren();
      links.forEach(function (link, index) {
        const row = document.createElement("tr");
        row.classList.toggle("is-active", index === activeLinkIndex);

        const linkCell = document.createElement("td");
        linkCell.textContent = String(index + 1);
        row.appendChild(linkCell);

        const angleCell = document.createElement("td");
        const angleControl = document.createElement("div");
        const angleOutput = document.createElement("output");
        const angleInput = document.createElement("input");
        angleControl.className = "dh-summary-angle";
        angleOutput.textContent = link.theta.toFixed(0) + "\u00b0";
        angleInput.type = "range";
        angleInput.min = "-180";
        angleInput.max = "180";
        angleInput.step = "1";
        angleInput.value = String(link.theta);
        angleInput.setAttribute("aria-label", "Joint angle theta " + (index + 1));
        angleInput.addEventListener("input", function () {
          link.theta = Number(angleInput.value);
          angleOutput.textContent = link.theta.toFixed(0) + "\u00b0";
          activeLinkIndex = index;
          activeStep = 4;
          updateControls();
          updateLinkTabs();
          updateActions();
          Array.from(summaryBody.children).forEach(function (summaryRow, rowIndex) {
            summaryRow.classList.toggle("is-active", rowIndex === activeLinkIndex);
          });
          description.textContent = stepDescription(activeStep);
          renderPlot();
        });
        angleControl.appendChild(angleOutput);
        angleControl.appendChild(angleInput);
        angleCell.appendChild(angleControl);
        row.appendChild(angleCell);

        [
          link.d.toFixed(2),
          link.a.toFixed(2),
          link.alpha.toFixed(0) + "\u00b0"
        ].forEach(function (value) {
          const cell = document.createElement("td");
          cell.textContent = value;
          row.appendChild(cell);
        });
        summaryBody.appendChild(row);
      });
    }

    function applyDhStep(origin, rotation, parameters, step) {
      const afterTheta = step >= 1
        ? multiply3(rotation, rotationZ(radians(parameters.theta)))
        : rotation;
      const afterOffset = step >= 2
        ? addVectors(origin, multiplyVector(rotation, [0, 0, parameters.d]))
        : origin;
      const afterLength = step >= 3
        ? addVectors(afterOffset, multiplyVector(afterTheta, [parameters.a, 0, 0]))
        : afterOffset;
      const finalRotation = step >= 4
        ? multiply3(afterTheta, rotationX(radians(parameters.alpha)))
        : afterTheta;

      return {
        offset: afterOffset,
        endpoint: afterLength,
        rotation: finalRotation
      };
    }

    function plotRanges(points) {
      const minimum = [0, 1, 2].map(function (dimension) {
        return Math.min.apply(null, points.map(function (point) { return point[dimension]; }));
      });
      const maximum = [0, 1, 2].map(function (dimension) {
        return Math.max.apply(null, points.map(function (point) { return point[dimension]; }));
      });
      const largestSpan = Math.max(
        maximum[0] - minimum[0],
        maximum[1] - minimum[1],
        maximum[2] - minimum[2]
      );
      const halfRange = Math.max(1.35, largestSpan / 2 + 0.55);

      return [0, 1, 2].map(function (dimension) {
        const center = (minimum[dimension] + maximum[dimension]) / 2;
        return [center - halfRange, center + halfRange];
      });
    }

    function renderPlot() {
      if (typeof window.Plotly === "undefined") {
        plotRoot.innerHTML = '<p class="kinematics-fallback">The 3D figure could not load. The written DH steps below contain the same transformation sequence.</p>';
        return;
      }

      rememberCamera();
      const identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
      const traces = [];
      const visiblePoints = [[0, 0, 0]];
      let origin = [0, 0, 0];
      let rotation = identity;
      const lastVisibleLink = activeStep === 4 ? links.length - 1 : activeLinkIndex;

      addFrame(traces, origin, rotation, "<sub>0</sub>", activeLinkIndex === 0 && activeStep === 0 ? 1 : 0.55);
      traces.push(pointLabelTrace(origin, "frame {0}", "#4b5563"));

      for (let index = 0; index <= lastVisibleLink; index += 1) {
        const linkNumber = index + 1;
        const step = index === activeLinkIndex ? activeStep : 4;
        const result = applyDhStep(origin, rotation, links[index], step);

        traces.push(jointAxisTrace(
          origin,
          matrixColumn(rotation, 2),
          "#6b7280",
          index === activeLinkIndex ? "joint " + linkNumber + ": z<sub>" + index + "</sub>" : "",
          index === activeLinkIndex ? 0.95 : 0.5
        ));

        if (step >= 2) {
          traces.push(lineTrace([origin, result.offset], "#7c3aed", 8));
          visiblePoints.push(result.offset);
        }
        if (step >= 3) {
          traces.push(lineTrace([result.offset, result.endpoint], "#d97706", 9));
          visiblePoints.push(result.endpoint);
        }

        if (step > 0) {
          const completeFrame = step === 4;
          addFrame(
            traces,
            result.endpoint,
            result.rotation,
            completeFrame ? "<sub>" + linkNumber + "</sub>" : "'",
            index === activeLinkIndex ? 1 : 0.65
          );
          traces.push(pointLabelTrace(
            result.endpoint,
            completeFrame ? "frame {" + linkNumber + "}" : "intermediate frame",
            index === activeLinkIndex ? "#111827" : "#4b5563"
          ));
        }

        if (step < 4) {
          break;
        }
        origin = result.endpoint;
        rotation = result.rotation;
      }

      const ranges = plotRanges(visiblePoints);
      const layout = {
        margin: { l: 0, r: 0, b: 0, t: 0 },
        paper_bgcolor: "rgba(0,0,0,0)",
        font: { family: "system-ui, sans-serif", color: "#1f2937" },
        scene: {
          bgcolor: "#ffffff",
          aspectmode: "cube",
          uirevision: "dh-camera",
          xaxis: { title: "", range: ranges[0], showspikes: false, zeroline: false },
          yaxis: { title: "", range: ranges[1], showspikes: false, zeroline: false },
          zaxis: { title: "", range: ranges[2], showspikes: false, zeroline: false },
          camera: camera
        },
        uirevision: "dh-camera"
      };
      const config = {
        responsive: true,
        displaylogo: false,
        scrollZoom: true,
        modeBarButtonsToRemove: ["toImage", "sendDataToCloud"]
      };

      if (plotRoot.dataset.initialized) {
        window.Plotly.react(plotRoot, traces, layout, config);
      } else {
        window.Plotly.newPlot(plotRoot, traces, layout, config).then(function () {
          if (!plotRoot.dataset.cameraListener) {
            plotRoot.on("plotly_relayout", function (eventData) {
              if (eventData["scene.camera"]) {
                camera = copyCamera(eventData["scene.camera"]);
              }
            });
            plotRoot.dataset.cameraListener = "true";
          }
        });
        plotRoot.dataset.initialized = "true";
      }
    }

    function render() {
      updateControls();
      updateLinkTabs();
      updateActions();
      updateSummary();
      description.textContent = stepDescription(activeStep);
      renderPlot();
    }

    buttons.forEach(function (button) {
      button.addEventListener("click", function () {
        const step = Number(button.dataset.dhStep);
        activeStep = step;
        render();
      });
    });

    inputs.forEach(function (input) {
      input.addEventListener("input", function () {
        currentLink()[input.dataset.dhParam] = Number(input.value);
        render();
      });
    });

    addButton.addEventListener("click", function () {
      if (addButton.disabled) {
        return;
      }
      links.push(createLink(links.length));
      activeLinkIndex = links.length - 1;
      activeStep = 4;
      render();
    });

    removeButton.addEventListener("click", function () {
      if (removeButton.disabled) {
        return;
      }
      links.pop();
      activeLinkIndex = Math.min(activeLinkIndex, links.length - 1);
      activeStep = 4;
      render();
    });

    resetButton.addEventListener("click", function () {
      links = [createLink(0)];
      activeLinkIndex = 0;
      activeStep = 4;
      render();
    });

    render();
  }

  function createSvgElement(name, attributes) {
    const element = document.createElementNS(SVG_NS, name);
    Object.entries(attributes || {}).forEach(function (entry) {
      element.setAttribute(entry[0], String(entry[1]));
    });
    return element;
  }

  function appendSvg(parent, name, attributes, text) {
    const element = createSvgElement(name, attributes);
    if (text !== undefined) {
      element.textContent = text;
    }
    parent.appendChild(element);
    return element;
  }

  function planarTransform(theta, length) {
    const cosine = Math.cos(theta);
    const sine = Math.sin(theta);
    return [
      [cosine, -sine, 0, length * cosine],
      [sine, cosine, 0, length * sine],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];
  }

  function renderMatrix(container, matrix, label) {
    const table = document.createElement("table");
    table.className = "numeric-matrix";
    table.setAttribute("aria-label", label);
    const body = document.createElement("tbody");

    matrix.forEach(function (row) {
      const tableRow = document.createElement("tr");
      row.forEach(function (value) {
        const cell = document.createElement("td");
        cell.textContent = formatNumber(value);
        tableRow.appendChild(cell);
      });
      body.appendChild(tableRow);
    });

    table.appendChild(body);
    container.replaceChildren(table);
  }

  function initializePlanarDemo(root) {
    const svg = root.querySelector("svg");
    const inputs = Array.from(root.querySelectorAll("[data-planar-param]"));
    const center = [340, 210];
    let drawingScale = 90;

    function readParameters() {
      const values = {};
      inputs.forEach(function (input) {
        values[input.dataset.planarParam] = Number(input.value);
      });
      return values;
    }

    function toSvg(point) {
      return [center[0] + point[0] * drawingScale, center[1] - point[1] * drawingScale];
    }

    function drawLine(parent, start, end, attributes) {
      const startSvg = toSvg(start);
      const endSvg = toSvg(end);
      return appendSvg(parent, "line", Object.assign({
        x1: startSvg[0],
        y1: startSvg[1],
        x2: endSvg[0],
        y2: endSvg[1]
      }, attributes));
    }

    function drawFrame(parent, position, angle, suffix) {
      const frameLength = 0.40;
      const xEnd = [
        position[0] + frameLength * Math.cos(angle),
        position[1] + frameLength * Math.sin(angle)
      ];
      const yEnd = [
        position[0] - frameLength * Math.sin(angle),
        position[1] + frameLength * Math.cos(angle)
      ];
      drawLine(parent, position, xEnd, {
        class: "planar-frame-axis planar-frame-axis--x",
        "marker-end": "url(#planar-arrow-x)"
      });
      drawLine(parent, position, yEnd, {
        class: "planar-frame-axis planar-frame-axis--y",
        "marker-end": "url(#planar-arrow-y)"
      });
      const xText = toSvg(xEnd);
      const yText = toSvg(yEnd);
      appendSvg(parent, "text", { x: xText[0] + 5, y: xText[1] - 4, class: "planar-frame-label planar-frame-label--x" }, "x" + suffix);
      appendSvg(parent, "text", { x: yText[0] + 5, y: yText[1] - 4, class: "planar-frame-label planar-frame-label--y" }, "y" + suffix);

      const origin = toSvg(position);
      appendSvg(parent, "circle", { cx: origin[0], cy: origin[1], r: 7, class: "planar-z-axis" });
      appendSvg(parent, "circle", { cx: origin[0], cy: origin[1], r: 2.4, class: "planar-z-axis-dot" });
      appendSvg(parent, "text", { x: origin[0] + 9, y: origin[1] + 17, class: "planar-frame-name" }, "{" + suffix + "}");
    }

    function drawAngle(parent, centerPoint, startAngle, sweepAngle, radius, label) {
      const points = [];
      const steps = 24;
      for (let index = 0; index <= steps; index += 1) {
        const angle = startAngle + sweepAngle * (index / steps);
        points.push([
          centerPoint[0] + radius * Math.cos(angle),
          centerPoint[1] + radius * Math.sin(angle)
        ]);
      }
      const pathData = points.map(function (point, index) {
        const converted = toSvg(point);
        return (index === 0 ? "M " : "L ") + converted[0] + " " + converted[1];
      }).join(" ");
      appendSvg(parent, "path", { d: pathData, class: "planar-angle-arc" });

      const middleAngle = startAngle + sweepAngle / 2;
      const labelPoint = toSvg([
        centerPoint[0] + (radius + 0.14) * Math.cos(middleAngle),
        centerPoint[1] + (radius + 0.14) * Math.sin(middleAngle)
      ]);
      appendSvg(parent, "text", { x: labelPoint[0], y: labelPoint[1], class: "planar-angle-label" }, label);
    }

    function updateOutputs(parameters, tcp, orientation) {
      ["theta1", "theta2"].forEach(function (name) {
        root.querySelector('[data-planar-output="' + name + '"]').textContent = parameters[name].toFixed(0) + "\u00b0";
      });
      ["a1", "a2"].forEach(function (name) {
        root.querySelector('[data-planar-output="' + name + '"]').textContent = parameters[name].toFixed(2);
      });
      root.querySelector('[data-planar-pose="x"]').textContent = formatNumber(tcp[0]);
      root.querySelector('[data-planar-pose="y"]').textContent = formatNumber(tcp[1]);
      root.querySelector('[data-planar-pose="phi"]').textContent = (orientation * 180 / Math.PI).toFixed(0) + "\u00b0";
    }

    function render() {
      const parameters = readParameters();
      const theta1 = radians(parameters.theta1);
      const theta2 = radians(parameters.theta2);
      const theta12 = theta1 + theta2;
      drawingScale = Math.min(90, 185 / (parameters.a1 + parameters.a2));
      const origin = [0, 0];
      const joint = [
        parameters.a1 * Math.cos(theta1),
        parameters.a1 * Math.sin(theta1)
      ];
      const tcp = [
        joint[0] + parameters.a2 * Math.cos(theta12),
        joint[1] + parameters.a2 * Math.sin(theta12)
      ];
      const transform1 = planarTransform(theta1, parameters.a1);
      const transform2 = planarTransform(theta2, parameters.a2);
      const transform02 = multiply4(transform1, transform2);

      svg.replaceChildren();
      const definitions = appendSvg(svg, "defs");
      const markerX = appendSvg(definitions, "marker", {
        id: "planar-arrow-x", viewBox: "0 0 10 10", refX: 8, refY: 5,
        markerWidth: 5, markerHeight: 5, orient: "auto-start-reverse"
      });
      appendSvg(markerX, "path", { d: "M 0 0 L 10 5 L 0 10 z", fill: "#c2413b" });
      const markerY = appendSvg(definitions, "marker", {
        id: "planar-arrow-y", viewBox: "0 0 10 10", refX: 8, refY: 5,
        markerWidth: 5, markerHeight: 5, orient: "auto-start-reverse"
      });
      appendSvg(markerY, "path", { d: "M 0 0 L 10 5 L 0 10 z", fill: "#16865c" });

      const grid = appendSvg(svg, "g", { class: "planar-grid" });
      for (let value = -4; value <= 4; value += 1) {
        drawLine(grid, [-4.7, value], [4.7, value], {});
        drawLine(grid, [value, -3.2], [value, 3.2], {});
      }
      drawLine(grid, [-4.7, 0], [4.7, 0], { class: "planar-grid-axis" });
      drawLine(grid, [0, -3.2], [0, 3.2], { class: "planar-grid-axis" });

      const arm = appendSvg(svg, "g", { class: "planar-arm" });
      drawLine(arm, origin, joint, { class: "planar-link planar-link--one" });
      drawLine(arm, joint, tcp, { class: "planar-link planar-link--two" });
      [origin, joint, tcp].forEach(function (point, index) {
        const converted = toSvg(point);
        appendSvg(arm, "circle", {
          cx: converted[0],
          cy: converted[1],
          r: index === 2 ? 7 : 10,
          class: index === 2 ? "planar-tcp" : "planar-joint"
        });
      });

      const link1Label = toSvg([(origin[0] + joint[0]) / 2, (origin[1] + joint[1]) / 2]);
      const link2Label = toSvg([(joint[0] + tcp[0]) / 2, (joint[1] + tcp[1]) / 2]);
      appendSvg(arm, "text", { x: link1Label[0], y: link1Label[1] - 12, class: "planar-link-label" }, "a1");
      appendSvg(arm, "text", { x: link2Label[0], y: link2Label[1] - 12, class: "planar-link-label" }, "a2");

      const frames = appendSvg(svg, "g", { class: "planar-frames" });
      drawFrame(frames, origin, 0, "0");
      drawFrame(frames, joint, theta1, "1");
      drawFrame(frames, tcp, theta12, "2");
      drawAngle(frames, origin, 0, theta1, 0.42, "\u03b81");
      drawAngle(frames, joint, theta1, theta2, 0.34, "\u03b82");

      updateOutputs(parameters, tcp, theta12);
      renderMatrix(root.querySelector('[data-planar-matrix="t1"]'), transform1, "Transformation from frame 0 to frame 1");
      renderMatrix(root.querySelector('[data-planar-matrix="t2"]'), transform2, "Transformation from frame 1 to frame 2");
      renderMatrix(root.querySelector('[data-planar-matrix="t02"]'), transform02, "Transformation from frame 0 to frame 2");
    }

    inputs.forEach(function (input) {
      input.addEventListener("input", render);
    });
    render();
  }

  function initializeAll() {
    const dhRoot = document.getElementById("dh-transform-demo");
    const planarRoot = document.getElementById("planar-fk-demo");
    if (dhRoot) {
      initializeDhDemo(dhRoot);
    }
    if (planarRoot) {
      initializePlanarDemo(planarRoot);
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initializeAll);
  } else {
    initializeAll();
  }
})();
