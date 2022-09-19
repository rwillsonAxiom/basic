"use strict";
var __spreadArray = (this && this.__spreadArray) || function (to, from, pack) {
    if (pack || arguments.length === 2) for (var i = 0, l = from.length, ar; i < l; i++) {
        if (ar || !(i in from)) {
            if (!ar) ar = Array.prototype.slice.call(from, 0, i);
            ar[i] = from[i];
        }
    }
    return to.concat(ar || Array.prototype.slice.call(from));
};
var sgWorld = SGWorld;
var MathUtils;
(function (MathUtils) {
    var Vector = (function () {
        function Vector(data) {
            this.data = data;
        }
        Vector.prototype.copy = function () {
            return new Vector(__spreadArray([], this.data, true));
        };
        Vector.prototype.add = function (b) {
            this.data.forEach(function (value, index, array) { return array[index] = value + b.data[index]; });
            return this;
        };
        Vector.prototype.sub = function (b) {
            this.data.forEach(function (value, index, array) { return array[index] = value - b.data[index]; });
            return this;
        };
        Vector.prototype.mul = function (v) {
            this.data.forEach(function (value, index, array) { return array[index] = v * value; });
            return this;
        };
        Vector.prototype.dot = function (b) {
            return this.data.reduce(function (previous, value, index) { return previous + value * b.data[index]; }, 0);
        };
        Vector.prototype.cross = function (b) {
            if (this.data.length + 0 !== 3)
                throw new TypeError("Cross product only available for vectors of length 3");
            var data = this.data;
            return new Vector([
                data[1] * b.data[2] - b.data[1] * data[2],
                data[2] * b.data[0] - b.data[2] * data[0],
                data[0] * b.data[1] - b.data[0] * data[1]
            ]);
        };
        Vector.prototype.equals = function (b) {
            return this.data.every(function (value, index) { return value == b.data[index]; });
        };
        Vector.prototype.mag = function () {
            return Math.sqrt(this.data.reduce(function (previous, value) { return previous + value * value; }, 0));
        };
        Vector.prototype.normalise = function () {
            var mag = this.mag();
            if (mag == 0 && this.data.length > 0) {
                this.data[0] = 1;
                return this;
            }
            this.data.forEach(function (value, index, array) { return array[index] = value / mag; });
            return this;
        };
        return Vector;
    }());
    MathUtils.Vector = Vector;
    var Quaternion = (function () {
        function Quaternion(data) {
            this.data = data;
        }
        Quaternion.fromYPR = function (yRad, pRad, rRad) {
            var r = new Quaternion([0, 0, 0, 1]);
            var cy = Math.cos(yRad / 2.0);
            var sy = Math.sin(yRad / 2.0);
            var cp = Math.cos(pRad / 2.0);
            var sp = Math.sin(pRad / 2.0);
            var cr = Math.cos(rRad / 2.0);
            var sr = Math.sin(rRad / 2.0);
            r.data[0] = cy * sp * cr - sy * cp * sr;
            r.data[1] = cy * cp * sr + sy * sp * cr;
            r.data[2] = cy * sp * sr + sy * cp * cr;
            r.data[3] = cy * cp * cr - sy * sp * sr;
            return r;
        };
        Quaternion.prototype.copy = function () {
            return new Quaternion(__spreadArray([], this.data, true));
        };
        Quaternion.prototype.zeroRoll = function () {
            var pitch = this.getPitch();
            var yaw = this.getYaw();
            this.data = [0, 0, 0, 1].slice(0);
            this.postApplyXAxis(pitch).preApplyZAxis(yaw);
            this.normalise();
            return this;
        };
        Quaternion.prototype.getYPR = function () {
            var x = this.data[0];
            var y = this.data[1];
            var z = this.data[2];
            var w = this.data[3];
            var sinpitch = 2.0 * (z * y + w * x);
            if (Math.abs(sinpitch - 1) < 1e-5)
                return [
                    2 * Math.atan2(y, w),
                    Math.PI / 2,
                    0
                ];
            if (Math.abs(sinpitch + 1) < 1e-5)
                return [
                    -2 * Math.atan2(y, w),
                    -Math.PI / 2,
                    0
                ];
            return [
                Math.atan2(2.0 * (w * z - x * y), 1 - 2 * (x * x + z * z)),
                Math.asin(sinpitch),
                Math.atan2(2.0 * (w * y - x * z), 1 - 2 * (x * x + y * y))
            ];
        };
        Quaternion.prototype.getRoll = function () {
            return Math.atan2(2 * (this.data[3] * this.data[1] - this.data[0] * this.data[2]), 1 - 2 * (this.data[0] * this.data[0] + this.data[1] * this.data[1]));
        };
        Quaternion.prototype.getPitch = function () {
            var forward = this.getYAxis(1);
            var ret = Math.asin(forward.data[2]);
            if (isNaN(ret)) {
                return Math.PI / 2 * (forward.data[2] / Math.abs(forward.data[2]));
            }
            return ret;
        };
        Quaternion.prototype.getYaw = function () {
            var right = this.getXAxis(1);
            return Math.atan2(right.data[1], right.data[0]);
        };
        Quaternion.prototype.conjugate = function () {
            this.data = [-this.data[0], -this.data[1], -this.data[2], this.data[3]].slice(0);
            return this;
        };
        Quaternion.prototype.getXAxis = function (v) {
            return new Vector([
                v * (this.data[3] * this.data[3] + this.data[0] * this.data[0] - this.data[1] * this.data[1] - this.data[2] * this.data[2]),
                v * (2 * (this.data[0] * this.data[1] + this.data[2] * this.data[3])),
                v * (2 * (this.data[0] * this.data[2] - this.data[1] * this.data[3]))
            ]);
        };
        Quaternion.prototype.getYAxis = function (v) {
            return new Vector([
                v * (2 * (this.data[1] * this.data[0] - this.data[2] * this.data[3])),
                v * (this.data[3] * this.data[3] - this.data[0] * this.data[0] + this.data[1] * this.data[1] - this.data[2] * this.data[2]),
                v * (2 * (this.data[1] * this.data[2] + this.data[0] * this.data[3]))
            ]);
        };
        Quaternion.prototype.getZAxis = function (v) {
            return new Vector([
                v * (2 * (this.data[2] * this.data[0] + this.data[1] * this.data[3])),
                v * (2 * (this.data[2] * this.data[1] - this.data[0] * this.data[3])),
                v * (this.data[3] * this.data[3] - this.data[0] * this.data[0] - this.data[1] * this.data[1] + this.data[2] * this.data[2]),
            ]);
        };
        Quaternion.prototype.postApplyXAxis = function (v) {
            var s = Math.sin(v / 2);
            var c = Math.cos(v / 2);
            this.data = [
                this.data[0] * c + this.data[3] * s,
                this.data[1] * c + this.data[2] * s,
                this.data[2] * c - this.data[1] * s,
                this.data[3] * c - this.data[0] * s
            ].slice(0);
            return this;
        };
        Quaternion.prototype.postApplyYAxis = function (v) {
            var s = Math.sin(v / 2);
            var c = Math.cos(v / 2);
            this.data = [
                this.data[0] * c - this.data[2] * s,
                this.data[1] * c + this.data[3] * s,
                this.data[2] * c + this.data[0] * s,
                this.data[3] * c - this.data[1] * s
            ].slice(0);
            return this;
        };
        Quaternion.prototype.postApplyZAxis = function (v) {
            var s = Math.sin(v / 2);
            var c = Math.cos(v / 2);
            this.data = [
                this.data[0] * c + this.data[1] * s,
                this.data[1] * c - this.data[0] * s,
                this.data[2] * c + this.data[3] * s,
                this.data[3] * c - this.data[2] * s
            ].slice(0);
            return this;
        };
        Quaternion.prototype.preApplyXAxis = function (v) {
            var s = Math.sin(v / 2);
            var c = Math.cos(v / 2);
            this.data = [
                this.data[0] * c + this.data[3] * s,
                this.data[1] * c - this.data[2] * s,
                this.data[2] * c + this.data[1] * s,
                this.data[3] * c - this.data[0] * s
            ].slice(0);
            return this;
        };
        Quaternion.prototype.preApplyYAxis = function (v) {
            var s = Math.sin(v / 2);
            var c = Math.cos(v / 2);
            this.data = [
                this.data[0] * c + this.data[2] * s,
                this.data[1] * c + this.data[3] * s,
                this.data[2] * c - this.data[0] * s,
                this.data[3] * c - this.data[1] * s
            ].slice(0);
            return this;
        };
        Quaternion.prototype.preApplyZAxis = function (v) {
            var s = Math.sin(v / 2);
            var c = Math.cos(v / 2);
            this.data = [
                this.data[0] * c - this.data[1] * s,
                this.data[1] * c + this.data[0] * s,
                this.data[2] * c + this.data[3] * s,
                this.data[3] * c - this.data[2] * s
            ].slice(0);
            return this;
        };
        Quaternion.prototype.apply = function (v) {
            var u = new Vector([this.data[0], this.data[1], this.data[2]]);
            var crossUV = u.cross(v);
            return v.copy().add(crossUV.copy().mul(this.data[3]).add(u.cross(crossUV)).mul(2));
        };
        Quaternion.prototype.mul = function (q) {
            this.data = [
                this.data[3] * q.data[0] + q.data[3] * this.data[0] + this.data[1] * q.data[2] - this.data[2] * q.data[1],
                this.data[3] * q.data[1] + q.data[3] * this.data[1] + this.data[2] * q.data[0] - this.data[0] * q.data[2],
                this.data[3] * q.data[2] + q.data[3] * this.data[2] + this.data[0] * q.data[1] - this.data[1] * q.data[0],
                this.data[3] * q.data[3] - this.data[0] * q.data[0] - this.data[1] * q.data[1] - this.data[2] * q.data[2]
            ].slice(0);
            return this;
        };
        Quaternion.prototype.equals = function (b) {
            return this.data.every(function (value, index) { return value == b.data[index]; });
        };
        Quaternion.prototype.mag = function () {
            return Math.sqrt(this.data.reduce(function (previous, value) { return previous + value * value; }, 0));
        };
        Quaternion.prototype.normalise = function () {
            var mag = this.mag();
            if (mag == 0 && this.data.length > 0) {
                this.data[0] = 1;
                return this;
            }
            this.data.forEach(function (value, index, array) { return array[index] = value / mag; });
            return this;
        };
        return Quaternion;
    }());
    MathUtils.Quaternion = Quaternion;
    function degsToRads(degs) { return degs / 180 * Math.PI; }
    MathUtils.degsToRads = degsToRads;
    function radsToDegs(degs) { return degs / Math.PI * 180; }
    MathUtils.radsToDegs = radsToDegs;
})(MathUtils || (MathUtils = {}));
var Axiom;
(function (Axiom) {
    var TextElement = (function () {
        function TextElement() {
            this.element = document.createElement("div");
            document.body.appendChild(this.element);
        }
        TextElement.prototype.writeLine = function (message) {
            this.element.textContent += message + "\n";
        };
        TextElement.prototype.setText = function (message) {
            this.element.textContent = message;
        };
        return TextElement;
    }());
    Axiom.TextElement = TextElement;
    function worldToRoomCoord(position) {
        var _a;
        var pos = sgWorld.SetParamEx(9013, position);
        var originalOri = MathUtils.Quaternion.fromYPR(MathUtils.degsToRads(pos.Yaw), MathUtils.degsToRads(pos.Pitch), MathUtils.degsToRads(-pos.Roll));
        var worldIPos = sgWorld.Navigate.GetPosition(3);
        var worldOri = MathUtils.Quaternion.fromYPR(MathUtils.degsToRads(worldIPos.Yaw), MathUtils.degsToRads(worldIPos.Pitch + (Program.instance.getDeviceType() === 1 ? 0 : 90)), MathUtils.degsToRads(-worldIPos.Roll));
        var newOri = worldOri.conjugate().mul(originalOri);
        var newYPR = newOri.getYPR();
        var ret = sgWorld.Creator.CreatePosition(pos.X, pos.Y, pos.Altitude, 3, MathUtils.radsToDegs(newYPR[0]), MathUtils.radsToDegs(newYPR[1]), MathUtils.radsToDegs(newYPR[2]), pos.Distance);
        _a = [ret.Altitude, ret.Y], ret.Y = _a[0], ret.Altitude = _a[1];
        return ret;
    }
    Axiom.worldToRoomCoord = worldToRoomCoord;
    function roomToWorldCoord(position) {
        var _a, _b;
        _a = [position.Altitude, position.Y], position.Y = _a[0], position.Altitude = _a[1];
        var pos = sgWorld.SetParamEx(9014, position);
        var originalOri = MathUtils.Quaternion.fromYPR(MathUtils.degsToRads(pos.Yaw), MathUtils.degsToRads(pos.Pitch), MathUtils.degsToRads(pos.Roll));
        var worldIPos = sgWorld.Navigate.GetPosition(3);
        var worldOri = MathUtils.Quaternion.fromYPR(MathUtils.degsToRads(worldIPos.Yaw), MathUtils.degsToRads(worldIPos.Pitch + (Program.instance.getDeviceType() === 1 ? 0 : 90)), MathUtils.degsToRads(-worldIPos.Roll));
        var newOri = worldOri.mul(originalOri);
        var newYPR = newOri.getYPR();
        var ret;
        _b = [sgWorld.Creator.CreatePosition(pos.X, pos.Y, pos.Altitude, 3, MathUtils.radsToDegs(newYPR[0]), MathUtils.radsToDegs(newYPR[1]), MathUtils.radsToDegs(-newYPR[2]), pos.Distance), position.Y, position.Altitude], ret = _b[0], position.Altitude = _b[1], position.Y = _b[2];
        return ret;
    }
    Axiom.roomToWorldCoord = roomToWorldCoord;
    function setComClientForcedInputMode() {
        sgWorld.SetParam(8166, 1);
    }
    Axiom.setComClientForcedInputMode = setComClientForcedInputMode;
    function getObject(oid, objectType) {
        if (oid !== undefined)
            try {
                var object = sgWorld.Creator.GetObject(oid);
                if (object.ObjectType === objectType)
                    return object;
            }
            catch (error) {
                return null;
            }
        return null;
    }
    Axiom.getObject = getObject;
    function getGroupID(groupName, parentGroup) {
        return sgWorld.ProjectTree.FindItem(groupName) || sgWorld.ProjectTree.CreateGroup(groupName, parentGroup);
    }
    Axiom.getGroupID = getGroupID;
    function deleteGroup(groupName) {
        var groupId = sgWorld.ProjectTree.FindItem(groupName);
        if (groupId) {
            sgWorld.ProjectTree.DeleteItem(groupId);
            return true;
        }
        return false;
    }
    Axiom.deleteGroup = deleteGroup;
    function deleteItemSafe(id) {
        if (id === undefined)
            return;
        try {
            var object = sgWorld.Creator.GetObject(id);
            if (object)
                sgWorld.Creator.DeleteObject(id);
        }
        catch (error) {
        }
    }
    Axiom.deleteItemSafe = deleteItemSafe;
    function highlightById(highlight, oid) {
        var model = getObject(oid, 17);
        if (model !== null)
            model.Terrain.Tint = sgWorld.Creator.CreateColor(0, 0, 0, highlight ? 50 : 0);
    }
    Axiom.highlightById = highlightById;
    var Ray = (function () {
        function Ray(groupID) {
            this.groupID = groupID;
        }
        Ray.prototype.draw = function (pickRayInfo) {
            var verticesArray = new Array(6);
            verticesArray[0] = pickRayInfo.originPoint.X;
            verticesArray[1] = pickRayInfo.originPoint.Y;
            verticesArray[2] = pickRayInfo.originPoint.Altitude;
            verticesArray[3] = pickRayInfo.hitPoint.X;
            verticesArray[4] = pickRayInfo.hitPoint.Y;
            verticesArray[5] = pickRayInfo.hitPoint.Altitude;
            if (this.id === undefined) {
                var rightRay = sgWorld.Creator.CreatePolylineFromArray(verticesArray, pickRayInfo.isNothing ? 0xFF0000FF : 0xFFFFFFFF, 3, this.groupID, "ray");
                rightRay.SetParam(200, 0x200);
                this.id = rightRay.ID;
            }
            else {
                try {
                    var obj = Axiom.getObject(this.id, 1);
                    if (obj !== null) {
                        obj.Geometry = sgWorld.Creator.GeometryCreator.CreateLineStringGeometry(verticesArray);
                        obj.LineStyle.Color.abgrColor = (pickRayInfo.objectID !== undefined) ? 0xFF0000FF : 0xFFFF0000;
                    }
                }
                catch (error) {
                    Program.instance.error("Ray error");
                }
            }
        };
        return Ray;
    }());
    Axiom.Ray = Ray;
    var Sphere = (function () {
        function Sphere(groupID) {
            this.groupID = groupID;
        }
        Sphere.prototype.draw = function (pickRayInfo) {
            var rayLengthScaleFactor = pickRayInfo.rayLength * 0.004;
            var sphereRadius = Math.max(0.01, rayLengthScaleFactor);
            var spherePivot = pickRayInfo.hitPoint.Copy();
            spherePivot.Altitude -= sphereRadius / 2;
            if (this.id == undefined) {
                var tip = sgWorld.Creator.CreateSphere(pickRayInfo.hitPoint.Copy(), sphereRadius, 0, 0x5000FF00, 0x5000FF00, 10, this.groupID, "rayTip");
                tip.SetParam(200, 0x200);
                this.id = tip.ID;
            }
            else {
                var obj = Axiom.getObject(this.id, 16);
                if (obj !== null) {
                    obj.Position = pickRayInfo.hitPoint.Copy();
                    obj.Position.Altitude -= sphereRadius / 2;
                    obj.SetParam(200, 0x200);
                    obj.Radius = sphereRadius;
                    obj.LineStyle.Color.FromARGBColor(pickRayInfo.objectID == undefined ? 0x50FFFFFF : 0x5000FF00);
                }
            }
        };
        return Sphere;
    }());
    Axiom.Sphere = Sphere;
    var Laser = (function () {
        function Laser(groupID) {
            this.groupID = groupID;
            this.ray = new Ray(this.groupID);
            this.tip = new Sphere(this.groupID);
        }
        Laser.prototype.update = function () {
            var position = Program.instance.controllerInfos[1].wandPosition;
            if (position === undefined)
                return;
            sgWorld.SetParam(8300, position);
            var hitObjectID = sgWorld.GetParam(8310);
            var distToHitPoint = sgWorld.GetParam(8312);
            var isNothing = false;
            if (distToHitPoint == 0) {
                distToHitPoint = sgWorld.Navigate.GetPosition(3).Altitude / 2;
                isNothing = true;
            }
            var hitPosition = position.Copy().Move(distToHitPoint, position.Yaw, position.Pitch);
            hitPosition.Cartesian = true;
            this.collision = {
                originPoint: position,
                hitPoint: hitPosition,
                rayLength: distToHitPoint,
                objectID: hitObjectID,
                isNothing: isNothing
            };
        };
        Laser.prototype.draw = function () {
            if (this.collision === undefined)
                return;
            this.ray.draw(this.collision);
            this.tip.draw(this.collision);
        };
        return Laser;
    }());
    Axiom.Laser = Laser;
    var Button = (function () {
        function Button(name, roomPositions, modelPath, callback, groupID, tooltip) {
            if (groupID === void 0) { groupID = ""; }
            if (tooltip === void 0) { tooltip = ""; }
            var _this = this;
            this.name = name;
            this.roomPositions = roomPositions;
            this.modelPath = modelPath;
            this.callback = callback;
            this.groupID = groupID;
            this.tooltip = tooltip;
            this.scale = 0.1;
            var obj = (function () {
                try {
                    return sgWorld.Creator.CreateModel(SGWorld.Creator.CreatePosition(0, 0, 0, 3), _this.modelPath, _this.scale, 0, _this.groupID, _this.name);
                }
                catch (_a) {
                    throw new Error("Failed to create model with filename \"".concat(modelPath, "\" while attempting to construct button \"").concat(name, "\""));
                }
            })();
            obj.BestLOD = 0;
            obj.Tooltip.Text = this.tooltip;
            obj.Visibility.Show = false;
            this.id = obj.ID;
        }
        Button.prototype.update = function () {
            if (this.id === Program.instance.getCollisionID() && Program.instance.getButton1Pressed()) {
                this.callback();
                Program.instance.setButton1Pressed(false);
            }
        };
        Button.prototype.draw = function () {
            var _a;
            var roomPos = this.roomPositions[Program.instance.getDeviceType()];
            if (roomPos === null)
                return;
            var pos = Axiom.roomToWorldCoord(roomPos);
            var obj = Axiom.getObject(this.id, 17);
            if (obj === null)
                return;
            obj.Position = pos;
            obj.ScaleFactor = this.scale * ((_a = Program.instance.controllerInfos[1].scaleFactor) !== null && _a !== void 0 ? _a : 1.5);
            obj.Visibility.Show = true;
        };
        Button.prototype.setScale = function (scale) {
            this.scale = scale;
        };
        Button.prototype.show = function (value) {
            if (this.id === undefined)
                this.draw();
            if (this.id === undefined)
                return;
            var obj = Axiom.getObject(this.id, 17);
            if (obj !== null)
                obj.Visibility.Show = value;
        };
        Button.prototype.destroy = function () {
            Axiom.deleteItemSafe(this.id);
        };
        return Button;
    }());
    Axiom.Button = Button;
    var Measurements = (function () {
        function Measurements() {
        }
        Measurements.init = function () {
            Measurements.group = getGroupID("measurements", getGroupID("Axiom"));
        };
        Measurements.measure = function () {
            var _a, _b;
            if (Program.instance.getButton2Pressed()) {
                deleteItemSafe(Measurements.lineID);
                deleteItemSafe(Measurements.labelID);
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[0].id);
                return;
            }
            if (Measurements.first === undefined || Measurements.lineID === undefined || Measurements.labelID === undefined) {
                Program.instance.error("measurement info missing");
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[0].id);
                return;
            }
            var end = (_b = (_a = Program.instance.laser) === null || _a === void 0 ? void 0 : _a.collision) === null || _b === void 0 ? void 0 : _b.hitPoint.Copy();
            if (end === undefined) {
                Program.instance.error("Lasert hitPoint missing");
                return;
            }
            var start = Measurements.first.Copy().AimTo(end);
            var line = getObject(Measurements.lineID, 1);
            if (line === null) {
                Program.instance.errorText.writeLine("measurment line can't be found");
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[0].id);
                return;
            }
            var geom = line.Geometry;
            geom.StartEdit();
            geom.Points.Item(1).X = end.X;
            geom.Points.Item(1).Y = end.Y;
            geom.EndEdit();
            var direction = start.Yaw.toFixed(2);
            var distance = start.DistanceTo(end).toFixed(2);
            var labelText = "".concat(direction, " ").concat(String.fromCharCode(176), " / ").concat(distance, "m");
            var halfPos = start.Move(start.DistanceTo(end) / 2, start.Yaw, 0);
            var label = getObject(Measurements.labelID, 18);
            if (label === null)
                return;
            label.Text = labelText;
            label.Position = halfPos;
            if (Program.instance.getButton1Pressed()) {
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[0].id);
                Program.instance.setButton1Pressed(false);
                Measurements.lineID = undefined;
                Measurements.labelID = undefined;
                Measurements.first = undefined;
            }
        };
        Measurements.measureStart = function () {
            var _a, _b;
            if (Program.instance.getButton2Pressed()) {
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[0].id);
                return;
            }
            if (!Program.instance.getButton1Pressed())
                return;
            Measurements.first = (_b = (_a = Program.instance.laser) === null || _a === void 0 ? void 0 : _a.collision) === null || _b === void 0 ? void 0 : _b.hitPoint.Copy();
            if (Measurements.first === undefined)
                return;
            var start = Measurements.first.Copy();
            var end = start.Copy();
            var lineWKT = "LineString( ".concat(start.X, " ").concat(start.Y, ", ").concat(end.X, " ").concat(end.Y, " )");
            var lineGeom = sgWorld.Creator.GeometryCreator.CreateLineStringGeometry(lineWKT);
            var group = Measurements.group;
            if (group === undefined) {
                Program.instance.error("no measurement group");
                return;
            }
            var line = sgWorld.Creator.CreatePolyline(lineGeom, Measurements.lineColor, 2, group, "line");
            line.LineStyle.Width = -10;
            var label = sgWorld.Creator.CreateTextLabel(start, "0m", Measurements.labelStyle, group, "label");
            Measurements.lineID = line.ID;
            Measurements.labelID = label.ID;
            Program.instance.setButton1Pressed(false);
            Program.instance.interactionMode = 2;
        };
        Measurements.lineColor = sgWorld.Creator.CreateColor(255, 255, 0, 255);
        Measurements.labelStyle = sgWorld.Creator.CreateLabelStyle(0);
        (function () {
            Measurements.labelStyle.PivotAlignment = "Top";
            Measurements.labelStyle.MultilineJustification = "Left";
        })();
        return Measurements;
    }());
    Axiom.Measurements = Measurements;
    var Drawings = (function () {
        function Drawings() {
        }
        Drawings.init = function () {
            Drawings.group = getGroupID("drawings", getGroupID("Axiom"));
        };
        Drawings.line = function () {
            var _a, _b, _c;
            if (Drawings.previous === undefined || Drawings.lineID === undefined) {
                Program.instance.errorText.writeLine("drawings info missing");
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[1].id);
                return;
            }
            var end = (_b = (_a = Program.instance.laser) === null || _a === void 0 ? void 0 : _a.collision) === null || _b === void 0 ? void 0 : _b.hitPoint.Copy();
            if (end === undefined) {
                Program.instance.errorText.writeLine("Lasert hitPoint missing");
                return;
            }
            var line = getObject(Drawings.lineID, 1);
            if (line === null) {
                Program.instance.errorText.writeLine("drawing line can't be found");
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[1].id);
                return;
            }
            var geom = line.Geometry;
            geom.StartEdit();
            var drawPointIndex = geom.Points.Count - 1;
            geom.Points.Item(drawPointIndex).X = end.X;
            geom.Points.Item(drawPointIndex).Y = end.Y;
            if (Program.instance.getButton1Pressed()) {
                geom.Points.AddPoint(end.X, end.Y, end.Altitude);
                Drawings.previous = end;
            }
            else if (Program.instance.getButton1()) {
                if (end.DistanceTo(Drawings.previous) / ((_c = Program.instance.controllerInfos[1].scaleFactor) !== null && _c !== void 0 ? _c : 1) > 0.1) {
                    geom.Points.AddPoint(end.X, end.Y, end.Altitude);
                    Drawings.previous = end;
                }
            }
            else if (Program.instance.getButton2Pressed()) {
                geom.Points.DeletePoint(drawPointIndex);
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[1].id);
            }
            geom.EndEdit();
            if (geom.Points.Count === 1)
                deleteItemSafe(Drawings.lineID);
        };
        Drawings.lineStart = function () {
            var _a, _b;
            if (Program.instance.getButton2Pressed()) {
                Program.instance.interactionMode = 0;
                Axiom.highlightById(false, Program.instance.buttons[1].id);
                return;
            }
            if (!Program.instance.getButton1Pressed())
                return;
            Drawings.previous = (_b = (_a = Program.instance.laser) === null || _a === void 0 ? void 0 : _a.collision) === null || _b === void 0 ? void 0 : _b.hitPoint.Copy();
            if (Drawings.previous === undefined)
                return;
            var start = Drawings.previous.Copy();
            var end = start.Copy();
            var lineWKT = "LineString( " + start.X + " " + start.Y + ", " + end.X + " " + end.Y + " )";
            var geom = sgWorld.Creator.GeometryCreator.CreateLineStringGeometry(lineWKT);
            var group = Drawings.group;
            if (group === undefined) {
                Program.instance.error("no drawings group");
                return;
            }
            var line = sgWorld.Creator.CreatePolyline(geom, Drawings.lineColor, 2, group, "__line");
            line.LineStyle.Width = -10;
            Drawings.lineID = line.ID;
            Program.instance.setButton1Pressed(false);
            Program.instance.interactionMode = 4;
        };
        Drawings.lineColor = sgWorld.Creator.CreateColor(0, 0, 0, 0);
        return Drawings;
    }());
    Axiom.Drawings = Drawings;
})(Axiom || (Axiom = {}));
var Program = (function () {
    function Program(folder) {
        this.folder = folder;
        this.hasFolder = false;
        this.errorText = new Axiom.TextElement();
        this.staticText = new Axiom.TextElement();
        this.runningText = new Axiom.TextElement();
        this.deviceType = 2;
        this.controllerInfos = [{}, {}];
        this.buttons = [];
        this.interactionMode = 0;
        this.runningText.writeLine("folder = \"".concat(folder, "\""));
        this.init();
    }
    Program.instantiate = function () {
        var _a;
        var folder = (_a = window.location.pathname.match(/^[\/\\]*(.*)[\/\\]/)) === null || _a === void 0 ? void 0 : _a[1];
        if (folder === undefined) {
            var div_1 = document.createElement("div");
            var label = document.createElement("label");
            var input_1 = document.createElement("input");
            var button = document.createElement("button");
            label.textContent = "Please manually enter the folder";
            alert("Please manually enter the folder");
            input_1.focus();
            button.textContent = "Confirm";
            button.onclick = function () {
                var _a;
                (_a = div_1.parentNode) === null || _a === void 0 ? void 0 : _a.removeChild(div_1);
                Program.instance = new Program(input_1.value);
            };
            div_1.appendChild(label);
            label.appendChild(input_1);
            div_1.appendChild(button);
            document.body.appendChild(div_1);
            return;
        }
        Program.instance = new Program(folder);
    };
    Program.prototype.error = function (message) {
        this.errorText.writeLine(message);
        if (this.onSGWorld !== undefined) {
            sgWorld.DetachEvent("OnSGWorld", this.onSGWorld);
            this.onSGWorld = undefined;
        }
        throw new Error("Program Error Called");
    };
    Program.prototype.update = function () {
        var _this = this;
        var _a, _b, _c, _d, _e, _f, _g, _h;
        var VRCstr = sgWorld.GetParam(8600);
        var VRControllersInfo = (function () {
            try {
                return JSON.parse(VRCstr);
            }
            catch (_a) {
                _this.error("ControllerInfo JSON parse error");
            }
        })();
        if (VRControllersInfo !== undefined) {
            for (var hand = 0; hand < 2; ++hand) {
                var prevTrigger = (_b = (_a = this.controllerInfos[hand]) === null || _a === void 0 ? void 0 : _a.trigger) !== null && _b !== void 0 ? _b : false;
                var prevButton1 = (_d = (_c = this.controllerInfos[hand]) === null || _c === void 0 ? void 0 : _c.button1) !== null && _d !== void 0 ? _d : false;
                var prevButton2 = (_f = (_e = this.controllerInfos[hand]) === null || _e === void 0 ? void 0 : _e.button2) !== null && _f !== void 0 ? _f : false;
                this.controllerInfos[hand] = {};
                var triggerOn = VRControllersInfo.IndexTrigger && VRControllersInfo.IndexTrigger[hand] != 0;
                var button1On = (VRControllersInfo.Buttons & [0x200, 0x2][hand]) != 0;
                var button2On = (VRControllersInfo.Buttons & [0x100, 0x1][hand]) != 0;
                this.controllerInfos[hand].triggerPressed = triggerOn && !prevTrigger;
                this.controllerInfos[hand].button1Pressed = button1On && !prevButton1;
                this.controllerInfos[hand].button2Pressed = button2On && !prevButton2;
                this.controllerInfos[hand].trigger = triggerOn;
                this.controllerInfos[hand].button1 = button1On;
                this.controllerInfos[hand].button2 = button2On;
                if (VRControllersInfo.HavePosition != undefined && VRControllersInfo.HavePosition[hand]) {
                    var wandPosition = sgWorld.Navigate.GetPosition(3);
                    wandPosition.Distance = 100000;
                    wandPosition.X = VRControllersInfo.Position[hand][0];
                    wandPosition.Y = VRControllersInfo.Position[hand][2];
                    wandPosition.Altitude = VRControllersInfo.Position[hand][1];
                    if (VRControllersInfo.HaveOrientation != undefined && VRControllersInfo.HaveOrientation[hand]) {
                        wandPosition.Yaw = VRControllersInfo.Yaw[hand];
                        wandPosition.Pitch = VRControllersInfo.Pitch[hand];
                        wandPosition.Roll = VRControllersInfo.Roll[hand];
                        var headPosition = sgWorld.Navigate.GetPosition(3);
                        var tmpHeadsetpos = sgWorld.GetParam(8601);
                        headPosition.X = tmpHeadsetpos.X;
                        headPosition.Y = tmpHeadsetpos.Y;
                        headPosition.Altitude = tmpHeadsetpos.Altitude;
                        headPosition.Yaw = tmpHeadsetpos.Yaw;
                        headPosition.Pitch = tmpHeadsetpos.Pitch;
                        headPosition.Roll = tmpHeadsetpos.Roll;
                        this.controllerInfos[hand].headPosition = headPosition;
                    }
                    this.controllerInfos[hand].wandPosition = wandPosition;
                }
                this.controllerInfos[hand].scaleFactor = VRControllersInfo.ScaleFactor;
            }
        }
        else {
            this.controllerInfos[0].triggerPressed = false;
            this.controllerInfos[0].button1Pressed = false;
            this.controllerInfos[0].button2Pressed = false;
            this.controllerInfos[1].triggerPressed = false;
            this.controllerInfos[1].button1Pressed = false;
            this.controllerInfos[1].button2Pressed = false;
        }
        if (this.roomExtent === undefined) {
            var extent = sgWorld.SetParamEx(9015);
            var roomExtent = JSON.parse(extent);
            this.roomExtent = {
                min: new MathUtils.Vector([roomExtent.minX, roomExtent.minY, roomExtent.minZ]),
                max: new MathUtils.Vector([roomExtent.maxX, roomExtent.maxY, roomExtent.maxZ])
            };
        }
        (_g = this.laser) === null || _g === void 0 ? void 0 : _g.update();
        for (var _i = 0, _j = this.buttons; _i < _j.length; _i++) {
            var button = _j[_i];
            button.draw();
            button.update();
        }
        (_h = this.laser) === null || _h === void 0 ? void 0 : _h.draw();
        this.dragMode();
        this.interact();
    };
    Program.prototype.tableFrame = function (eventID) {
        try {
            if (eventID == 14)
                this.update();
        }
        catch (_a) {
            this.error("Error on table frame");
        }
    };
    Program.prototype.firstTableFrame = function (eventID) {
        try {
            this.runningText.writeLine("first frame");
            if (eventID == 14) {
                Axiom.setComClientForcedInputMode();
                sgWorld.DetachEvent("OnSGWorld", this.onSGWorld);
                this.onSGWorld = this.tableFrame.bind(this);
                sgWorld.AttachEvent("OnSGWorld", this.onSGWorld);
            }
        }
        catch (_a) {
            this.error("Error on first table frame");
        }
    };
    Program.prototype.init = function () {
        this.runningText.writeLine("init");
        Axiom.deleteGroup("Axiom");
        Axiom.Measurements.init();
        Axiom.Drawings.init();
        this.createLaser();
        this.createButtons();
        this.onSGWorld = this.firstTableFrame.bind(this);
        sgWorld.AttachEvent("OnSGWorld", this.onSGWorld);
    };
    Program.prototype.createLaser = function () {
        this.laser = new Axiom.Laser(Axiom.getGroupID("Laser", Axiom.getGroupID("Axiom")));
    };
    Program.prototype.createButtons = function () {
        var _a, _b;
        var group = Axiom.getGroupID("Buttons", Axiom.getGroupID("Axiom"));
        this.buttons.push(new Axiom.Button("Measure", (_a = {},
            _a[2] = null,
            _a[0] = sgWorld.Creator.CreatePosition(-0.5, -1.1, 0.65, 3),
            _a[1] = sgWorld.Creator.CreatePosition(-0.9, 0, 1.1, 3),
            _a), this.folder + "/measure.xpl2", this.pushMeasure.bind(this), group));
        this.buttons.push(new Axiom.Button("Line", (_b = {},
            _b[2] = null,
            _b[0] = sgWorld.Creator.CreatePosition(-0.4, -1.1, 0.65, 3),
            _b[1] = sgWorld.Creator.CreatePosition(-0.9, 0, 1, 3),
            _b), this.folder + "/line.xpl2", this.pushLine.bind(this), group));
    };
    Program.prototype.pushMeasure = function () {
        if (this.interactionMode !== 0)
            return;
        Axiom.highlightById(true, this.buttons[0].id);
        this.runningText.writeLine("Measure");
        this.interactionMode = 1;
    };
    Program.prototype.pushLine = function () {
        if (this.interactionMode !== 0)
            return;
        Axiom.highlightById(true, this.buttons[1].id);
        this.runningText.writeLine("Line");
        this.interactionMode = 3;
    };
    Program.prototype.interact = function () {
        switch (this.interactionMode) {
            case 1:
                Axiom.Measurements.measureStart();
                break;
            case 2:
                Axiom.Measurements.measure();
                break;
            case 3:
                Axiom.Drawings.lineStart();
                break;
            case 4:
                Axiom.Drawings.line();
                break;
        }
    };
    Program.prototype.getButton3 = function () { var _a, _b; return (_b = (_a = this.controllerInfos[1]) === null || _a === void 0 ? void 0 : _a.trigger) !== null && _b !== void 0 ? _b : false; };
    Program.prototype.getCollisionPosition = function () { var _a, _b; return (_b = (_a = this.laser) === null || _a === void 0 ? void 0 : _a.collision) === null || _b === void 0 ? void 0 : _b.hitPoint; };
    Program.prototype.getWandPosition = function () { var _a, _b; return (_b = (_a = this.laser) === null || _a === void 0 ? void 0 : _a.collision) === null || _b === void 0 ? void 0 : _b.originPoint; };
    Program.prototype.getCollisionID = function () { var _a, _b; return (_b = (_a = this.laser) === null || _a === void 0 ? void 0 : _a.collision) === null || _b === void 0 ? void 0 : _b.objectID; };
    Program.prototype.getButton1Pressed = function () { var _a, _b; return (_b = (_a = this.controllerInfos[1]) === null || _a === void 0 ? void 0 : _a.button1Pressed) !== null && _b !== void 0 ? _b : false; };
    Program.prototype.getButton1 = function () { var _a, _b; return (_b = (_a = this.controllerInfos[1]) === null || _a === void 0 ? void 0 : _a.button1) !== null && _b !== void 0 ? _b : false; };
    Program.prototype.getButton2Pressed = function () { var _a, _b; return (_b = (_a = this.controllerInfos[1]) === null || _a === void 0 ? void 0 : _a.button2Pressed) !== null && _b !== void 0 ? _b : false; };
    Program.prototype.setButton1Pressed = function (pressed) { var _a; var _b; ((_a = (_b = this.controllerInfos)[1]) !== null && _a !== void 0 ? _a : (_b[1] = {})).button1Pressed = pressed; };
    Program.prototype.getDeviceType = function () {
        var _a, _b;
        if (this.deviceType === 2 && ((_a = this.roomExtent) === null || _a === void 0 ? void 0 : _a.max) !== undefined) {
            if (((_b = this.roomExtent) === null || _b === void 0 ? void 0 : _b.max.data[2]) > 1.9)
                this.deviceType = 1;
            else
                this.deviceType = 0;
        }
        return this.deviceType;
    };
    Program.prototype.dragMode = function () {
        var trigger = this.getButton3();
        var newIntersect = this.getCollisionPosition();
        var wandWorldIPos = this.getWandPosition();
        if (wandWorldIPos === undefined)
            throw new Error("Unable to find wand position");
        var wandRoomIPos = Axiom.worldToRoomCoord(wandWorldIPos);
        var wandOri1 = MathUtils.Quaternion.fromYPR(-MathUtils.degsToRads(wandRoomIPos.Yaw), MathUtils.degsToRads(wandRoomIPos.Pitch), MathUtils.degsToRads(wandRoomIPos.Roll));
        var wandOri = wandOri1;
        var wandRoomDir = wandOri.getYAxis(1);
        var wandRoomPos = new MathUtils.Vector([wandRoomIPos.X, wandRoomIPos.Y, wandRoomIPos.Altitude]);
        this.staticText.setText(wandRoomPos.data.map(function (x) { return x.toFixed(2); }).join(", "));
        if (newIntersect === undefined) {
            return;
        }
        var roomIntersect = Axiom.worldToRoomCoord(newIntersect);
        var roomX = roomIntersect.X;
        var roomY = this.getDeviceType() === 0 ? roomIntersect.Y : roomIntersect.Altitude;
        if (this.roomExtent !== undefined) {
            var minX = this.roomExtent.min.data[0];
            var maxX = this.roomExtent.max.data[0];
            var minY = this.roomExtent.min.data[this.getDeviceType() == 0 ? 1 : 2];
            var maxY = this.roomExtent.max.data[this.getDeviceType() == 0 ? 1 : 2];
            var deadzone = 1;
            if (roomX < minX - deadzone || roomX > maxX + deadzone || roomY < minY - deadzone || roomY > maxY + deadzone)
                return;
        }
        if (this.startInfo !== undefined) {
            if (!trigger) {
                this.startInfo = undefined;
                return;
            }
            var worldPos = sgWorld.Navigate.GetPosition(3).Copy();
            var wandPosDiff = wandRoomPos.copy().sub(this.startInfo.prevWandRoomPos);
            var magDifference = wandPosDiff.mag();
            if (magDifference > 0 && magDifference < 1) {
                var forwardOrBack = wandPosDiff.dot(this.startInfo.prevWandRoomDir);
                forwardOrBack = forwardOrBack >= 0 ? 1 : -1;
                var scaleRatio = 5;
                var degs = MathUtils.radsToDegs(Math.acos(Math.abs(wandPosDiff.copy().normalise().dot(this.startInfo.prevWandRoomDir.copy().normalise()))));
                var thresholdLower = 25;
                var thresholdUpper = 40;
                var thresholdRange = thresholdUpper - thresholdLower;
                var scalingRatio = 1 - Math.min(Math.max(degs, thresholdLower) - thresholdLower, thresholdRange) / thresholdRange;
                var power = forwardOrBack * scalingRatio * magDifference * 2;
                var powerStrength = 1;
                if (this.getDeviceType() == 1) {
                    var curAltitudeRatio = worldPos.Altitude / 2000;
                    powerStrength = Math.min(Math.max(curAltitudeRatio, 0.25), 1);
                }
                var factor = Math.pow(scaleRatio, power * powerStrength);
                worldPos.Altitude *= factor;
                newIntersect.Cartesian = true;
                newIntersect = newIntersect.MoveToward(sgWorld.Navigate.GetPosition(3), (1 - factor) * newIntersect.DistanceTo(sgWorld.Navigate.GetPosition(3)));
                var maxTableAltitude = 500000;
                var maxWallAltitude = 60000;
                if (this.getDeviceType() === 0 && worldPos.Altitude > maxTableAltitude) {
                    worldPos.Altitude = maxTableAltitude;
                }
                else if (this.getDeviceType() === 1 && worldPos.Altitude > maxWallAltitude) {
                    worldPos.Altitude = maxWallAltitude;
                }
                if (worldPos.Altitude < 250) {
                    worldPos.Altitude = 250;
                }
            }
            worldPos.X += this.startInfo.intersect.X - newIntersect.X;
            worldPos.Y += this.startInfo.intersect.Y - newIntersect.Y;
            this.startInfo.prevWandRoomPos = wandRoomPos;
            this.startInfo.prevWandRoomDir = wandRoomDir;
            sgWorld.Navigate.SetPosition(worldPos);
        }
        else if (trigger) {
            this.startInfo = {
                intersect: newIntersect,
                prevWandRoomPos: wandRoomPos,
                prevWandRoomDir: wandRoomDir
            };
        }
    };
    return Program;
}());
