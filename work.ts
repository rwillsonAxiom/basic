declare const SGWorld: ISGWorld;
const sgWorld = SGWorld;

namespace MathUtils {
  type Shift<A extends Array<any>> = ((...args: A) => void) extends ((...args: [A[0], ...infer R]) => void) ? R : never;

  type GrowExpRev<A extends Array<any>, N extends number, P extends Array<Array<any>>> = A['length'] extends N ? A : {
    0: GrowExpRev<[...A, ...P[0]], N, P>,
    1: GrowExpRev<A, N, Shift<P>>
  }[[...A, ...P[0]][N] extends undefined ? 0 : 1];

  type GrowExp<A extends Array<any>, N extends number, P extends Array<Array<any>>> = A['length'] extends N ? A : {
    0: GrowExp<[...A, ...A], N, [A, ...P]>,
    1: GrowExpRev<A, N, P>
  }[[...A, ...A][N] extends undefined ? 0 : 1];

  export type FixedSizeArray<T, N extends number> = N extends 0 ? [] : N extends 1 ? [T] : GrowExp<[T, T], N, [[T]]>

  export class Vector<N extends number> {
    data: FixedSizeArray<number, N>;

    /**
     * Takes ownership of the provided NumberTuple
     * @param data NumberTuple to use internally
     */
    constructor(data: FixedSizeArray<number, N>) {
      this.data = data;
    }

    copy(): Vector<N> {
      return new Vector<N>([...this.data]);
    }

    add(b: Vector<N>): Vector<N> {
      this.data.forEach((value, index, array) => array[index] = value + b.data[index]!);
      return this;
    }

    sub(b: Vector<N>): Vector<N> {
      this.data.forEach((value, index, array) => array[index] = value - b.data[index]!);
      return this;
    }

    mul(v: number): Vector<N> {
      this.data.forEach((value, index, array) => array[index] = v * value);
      return this;
    }

    dot(b: Vector<N>): number {
      return this.data.reduce((previous, value, index) => previous + value * b.data[index]!, 0);
    }

    cross(b: Vector<3>): Vector<3> {
      if (this.data.length + 0 !== 3)
        throw new TypeError("Cross product only available for vectors of length 3");
      const data = <FixedSizeArray<number, 3>>this.data;
      return new Vector([
        data[1] * b.data[2] - b.data[1] * data[2],
        data[2] * b.data[0] - b.data[2] * data[0],
        data[0] * b.data[1] - b.data[0] * data[1]
      ]);
    }

    equals(b: Vector<N>): boolean {
      return this.data.every((value, index) => value == b.data[index])
    }

    mag(): number {
      return Math.sqrt(this.data.reduce((previous, value) => previous + value * value, 0));
    }

    normalise(): Vector<N> {
      let mag = this.mag();
      if (mag == 0 && this.data.length > 0) {
        this.data[0] = 1;
        return this;
      }
      this.data.forEach((value, index, array) => array[index] = value / mag);
      return this;
    }
  }

  export class Quaternion {
    data: FixedSizeArray<number, 4>;

    /**
     * Takes ownership of the provided QuaternionArray
     * @param data QuaternionArray to use internally
     */
    constructor(data: FixedSizeArray<number, 4>) {
      this.data = data;
    }

    static fromYPR(yRad: number, pRad: number, rRad: number): Quaternion {
      const r: Quaternion = new Quaternion([0, 0, 0, 1]);

      const cy = Math.cos(yRad / 2.0); // cos(Yaw)
      const sy = Math.sin(yRad / 2.0); // sin(Yaw)
      const cp = Math.cos(pRad / 2.0); // cos(Pitch)
      const sp = Math.sin(pRad / 2.0); // sin(Pitch)
      const cr = Math.cos(rRad / 2.0); // cos(Roll)
      const sr = Math.sin(rRad / 2.0); // sin(Roll)

      r.data[0] = cy * sp * cr - sy * cp * sr;
      r.data[1] = cy * cp * sr + sy * sp * cr;
      r.data[2] = cy * sp * sr + sy * cp * cr;
      r.data[3] = cy * cp * cr - sy * sp * sr;

      return r;
    }

    copy(): Quaternion {
      return new Quaternion([...this.data]);
    }

    // set zero roll on this quaternion
    zeroRoll(): Quaternion {
      let pitch = this.getPitch();
      let yaw = this.getYaw();
      [...this.data] = [0, 0, 0, 1];
      this.postApplyXAxis(pitch).preApplyZAxis(yaw);
      this.normalise();
      return this;
    }

    getYPR() {
      const x = this.data[0];
      const y = this.data[1];
      const z = this.data[2];
      const w = this.data[3];

      const sinpitch = 2.0 * (z * y + w * x);
      if (Math.abs(sinpitch - 1) < 1e-5)
        return [
          2 * Math.atan2(y, w),
          Math.PI / 2,
          0
        ];
      if (Math.abs(sinpitch + 1) < 1e-5)
        return [
          - 2 * Math.atan2(y, w),
          -Math.PI / 2,
          0
        ];
      return [
        Math.atan2(2.0 * (w * z - x * y), 1 - 2 * (x * x + z * z)),
        Math.asin(sinpitch),
        Math.atan2(2.0 * (w * y - x * z), 1 - 2 * (x * x + y * y))
      ];
    }

    getRoll() {
      return Math.atan2(2 * (this.data[3] * this.data[1] - this.data[0] * this.data[2]), 1 - 2 * (this.data[0] * this.data[0] + this.data[1] * this.data[1]))
    }

    getPitch() {
      let forward = this.getYAxis(1);
      let ret = Math.asin(forward.data[2]);
      if (isNaN(ret)) {
        return Math.PI / 2 * (forward.data[2] / Math.abs(forward.data[2]));
      }
      return ret;
    }

    getYaw() {
      // We use right because up may be zero in xy
      let right = this.getXAxis(1);
      return Math.atan2(right.data[1], right.data[0]);
    }

    conjugate(): Quaternion {
      [...this.data] = [-this.data[0], -this.data[1], -this.data[2], this.data[3]];
      return this;
    }

    // find the new axis * v
    getXAxis(v: number): Vector<3> {
      return new Vector<3>([
        v * (this.data[3] * this.data[3] + this.data[0] * this.data[0] - this.data[1] * this.data[1] - this.data[2] * this.data[2]),
        v * (2 * (this.data[0] * this.data[1] + this.data[2] * this.data[3])),
        v * (2 * (this.data[0] * this.data[2] - this.data[1] * this.data[3]))
      ]);
    }

    getYAxis(v: number): Vector<3> {
      return new Vector<3>([
        v * (2 * (this.data[1] * this.data[0] - this.data[2] * this.data[3])),
        v * (this.data[3] * this.data[3] - this.data[0] * this.data[0] + this.data[1] * this.data[1] - this.data[2] * this.data[2]),
        v * (2 * (this.data[1] * this.data[2] + this.data[0] * this.data[3]))
      ]);
    }

    getZAxis(v: number): Vector<3> {
      return new Vector<3>([
        v * (2 * (this.data[2] * this.data[0] + this.data[1] * this.data[3])),
        v * (2 * (this.data[2] * this.data[1] - this.data[0] * this.data[3])),
        v * (this.data[3] * this.data[3] - this.data[0] * this.data[0] - this.data[1] * this.data[1] + this.data[2] * this.data[2]),
      ]);
    }

    // rotate around the new axis by v radians
    postApplyXAxis(v: number): Quaternion {
      let s = Math.sin(v / 2);
      let c = Math.cos(v / 2);
      [...this.data] = [
        this.data[0] * c + this.data[3] * s,
        this.data[1] * c + this.data[2] * s,
        this.data[2] * c - this.data[1] * s,
        this.data[3] * c - this.data[0] * s
      ];
      return this;
    }

    postApplyYAxis(v: number): Quaternion {
      let s = Math.sin(v / 2);
      let c = Math.cos(v / 2);
      [...this.data] = [
        this.data[0] * c - this.data[2] * s,
        this.data[1] * c + this.data[3] * s,
        this.data[2] * c + this.data[0] * s,
        this.data[3] * c - this.data[1] * s
      ];
      return this;
    }

    postApplyZAxis(v: number): Quaternion {
      let s = Math.sin(v / 2);
      let c = Math.cos(v / 2);
      [...this.data] = [
        this.data[0] * c + this.data[1] * s,
        this.data[1] * c - this.data[0] * s,
        this.data[2] * c + this.data[3] * s,
        this.data[3] * c - this.data[2] * s
      ];
      return this;
    }

    // rotate around the original axis by v radians
    preApplyXAxis(v: number): Quaternion {
      let s = Math.sin(v / 2);
      let c = Math.cos(v / 2);
      [...this.data] = [
        this.data[0] * c + this.data[3] * s,
        this.data[1] * c - this.data[2] * s,
        this.data[2] * c + this.data[1] * s,
        this.data[3] * c - this.data[0] * s
      ];
      return this;
    }

    preApplyYAxis(v: number): Quaternion {
      let s = Math.sin(v / 2);
      let c = Math.cos(v / 2);
      [...this.data] = [
        this.data[0] * c + this.data[2] * s,
        this.data[1] * c + this.data[3] * s,
        this.data[2] * c - this.data[0] * s,
        this.data[3] * c - this.data[1] * s
      ];
      return this;
    }

    preApplyZAxis(v: number): Quaternion {
      let s = Math.sin(v / 2);
      let c = Math.cos(v / 2);
      [...this.data] = [
        this.data[0] * c - this.data[1] * s,
        this.data[1] * c + this.data[0] * s,
        this.data[2] * c + this.data[3] * s,
        this.data[3] * c - this.data[2] * s
      ];
      return this;
    }

    apply(v: Vector<3>) {
      const u = new Vector<3>([this.data[0], this.data[1], this.data[2]]);
      const crossUV = u.cross(v);
      return v.copy().add(crossUV.copy().mul(this.data[3]).add(u.cross(crossUV)).mul(2));
    }

    mul(q: Quaternion) {
      [...this.data] = [
        this.data[3] * q.data[0] + q.data[3] * this.data[0] + this.data[1] * q.data[2] - this.data[2] * q.data[1],
        this.data[3] * q.data[1] + q.data[3] * this.data[1] + this.data[2] * q.data[0] - this.data[0] * q.data[2],
        this.data[3] * q.data[2] + q.data[3] * this.data[2] + this.data[0] * q.data[1] - this.data[1] * q.data[0],
        this.data[3] * q.data[3] - this.data[0] * q.data[0] - this.data[1] * q.data[1] - this.data[2] * q.data[2]
      ];
      return this;
    }

    equals(b: Quaternion): boolean {
      return this.data.every((value, index) => value == b.data[index])
    }

    mag(): number {
      return Math.sqrt(this.data.reduce((previous, value) => previous + value * value, 0));
    }

    normalise(): Quaternion {
      let mag = this.mag();
      if (mag == 0 && this.data.length > 0) {
        this.data[0] = 1;
        return this;
      }
      this.data.forEach((value, index, array) => array[index] = value / mag);
      return this;
    }
  }

  export function degsToRads(degs: number) { return degs / 180 * Math.PI; }
  export function radsToDegs(degs: number) { return degs / Math.PI * 180; }
}

namespace Axiom {
  export class TextElement {
    element: HTMLElement;

    constructor() {
      this.element = document.createElement("div");
      document.body.appendChild(this.element);
    }

    writeLine(message: string) {
      this.element.textContent += message + "\n";
    }
    setText(message: string) {
      this.element.textContent = message;
    }
  }

  export function worldToRoomCoord(position: IPosition) {
    let pos = sgWorld.SetParamEx(9013, position) as IPosition;
    const originalOri = MathUtils.Quaternion.fromYPR(MathUtils.degsToRads(pos.Yaw), MathUtils.degsToRads(pos.Pitch), MathUtils.degsToRads(-pos.Roll));
    const worldIPos = sgWorld.Navigate.GetPosition(3);
    const worldOri = MathUtils.Quaternion.fromYPR(MathUtils.degsToRads(worldIPos.Yaw), MathUtils.degsToRads(worldIPos.Pitch + (Program.instance!.getDeviceType() === DeviceType.wall ? 0 : 90)), MathUtils.degsToRads(-worldIPos.Roll));
    const newOri = worldOri.conjugate().mul(originalOri);
    const newYPR = newOri.getYPR();
    const ret = sgWorld.Creator.CreatePosition(pos.X, pos.Y, pos.Altitude, 3, MathUtils.radsToDegs(newYPR[0]), MathUtils.radsToDegs(newYPR[1]), MathUtils.radsToDegs(newYPR[2]), pos.Distance);

    [ret.Y, ret.Altitude] = [ret.Altitude, ret.Y];
    return ret;
  }

  export function roomToWorldCoord(position: IPosition) {
    [position.Y, position.Altitude] = [position.Altitude, position.Y];

    let pos = sgWorld.SetParamEx(9014, position) as IPosition;
    const originalOri = MathUtils.Quaternion.fromYPR(MathUtils.degsToRads(pos.Yaw), MathUtils.degsToRads(pos.Pitch), MathUtils.degsToRads(pos.Roll));
    const worldIPos = sgWorld.Navigate.GetPosition(3);
    const worldOri = MathUtils.Quaternion.fromYPR(MathUtils.degsToRads(worldIPos.Yaw), MathUtils.degsToRads(worldIPos.Pitch + (Program.instance!.getDeviceType() === DeviceType.wall ? 0 : 90)), MathUtils.degsToRads(-worldIPos.Roll));
    const newOri = worldOri.mul(originalOri);
    const newYPR = newOri.getYPR();

    let ret;
    [ret, position.Altitude, position.Y] = [sgWorld.Creator.CreatePosition(pos.X, pos.Y, pos.Altitude, 3, MathUtils.radsToDegs(newYPR[0]), MathUtils.radsToDegs(newYPR[1]), MathUtils.radsToDegs(-newYPR[2]), pos.Distance), position.Y, position.Altitude];
    return ret;
  }

  export function setComClientForcedInputMode() {
    sgWorld.SetParam(8166, 1); // Force COM input mode (Meaning your code here is in control)
  }

  export interface ControllerInfo {
    wandPosition: IPosition;
    button1: boolean;
    button2: boolean;
    button1Pressed: boolean;
    button2Pressed: boolean;
    trigger: boolean;
    triggerPressed: boolean;
    headPosition: IPosition;
    scaleFactor: number;
  }

  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_IMAGERY_LAYER): ITerrainRasterLayer | null;
  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_MODEL): ITerrainModel | null;
  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_POLYLINE): ITerrainPolyline | null;
  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_LABEL): ITerrainLabel | null;
  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_POLYGON): ITerrainPolygon | null;
  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_SPHERE): ITerrainSphere | null;
  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_BOX): ITerrain3DRectBase | null;
  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_IMAGE_LABEL): ITerrainImageLabel | null;
  export function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_RECTANGLE): ITerrainRectangle | null;

  export function getObject(oid: string | undefined, objectType: ObjectTypeCode): ITerraExplorerObject | null {
    if (oid !== undefined)
      try {
        const object = sgWorld.Creator.GetObject(oid);
        if (object.ObjectType === objectType)
          return object;
      } catch (error) {
        return null;
      }
    return null;
  }

  export function getGroupID(groupName: string, parentGroup?: string) {
    return sgWorld.ProjectTree.FindItem(groupName) || sgWorld.ProjectTree.CreateGroup(groupName, parentGroup);
  }

  export function deleteGroup(groupName: string) {
    const groupId = sgWorld.ProjectTree.FindItem(groupName);
    if (groupId) {
      sgWorld.ProjectTree.DeleteItem(groupId);
      return true;
    }
    return false;
  }

  export function deleteItemSafe(id: string | undefined) {
    if (id === undefined) return;
    try {
      const object = sgWorld.Creator.GetObject(id);
      if (object)
        sgWorld.Creator.DeleteObject(id);
    } catch (error) {
      // fine
    }
  }

  export function highlightById(highlight: boolean, oid: string) {
    const model = getObject(oid, ObjectTypeCode.OT_MODEL);
    if (model !== null)
      model.Terrain.Tint = sgWorld.Creator.CreateColor(0, 0, 0, highlight ? 50 : 0);
  }

  export const enum DeviceType {
    table,
    wall,
    desktop
  }

  export const enum Mode {
    None,
    MeasureStart,
    Measure,
    LineStart,
    Line
  }

  export interface ControllerInfo {
    wandPosition: IPosition;
    button1: boolean;
    button2: boolean;
    button1Pressed: boolean;
    button2Pressed: boolean;
    trigger: boolean;
    triggerPressed: boolean;
    headPosition: IPosition;
    scaleFactor: number;
  }

  export type collisionInfo = {
    originPoint: IPosition,
    hitPoint: IPosition,
    rayLength: number,
    objectID?: string,
    isNothing: boolean
  }

  export class Ray {
    constructor(private groupID: string) { }
    id?: string;
    draw(pickRayInfo: collisionInfo) {
      const verticesArray = new Array(6);
      verticesArray[0] = pickRayInfo.originPoint.X;
      verticesArray[1] = pickRayInfo.originPoint.Y;
      verticesArray[2] = pickRayInfo.originPoint.Altitude;
      verticesArray[3] = pickRayInfo.hitPoint.X;
      verticesArray[4] = pickRayInfo.hitPoint.Y;
      verticesArray[5] = pickRayInfo.hitPoint.Altitude;
      if (this.id === undefined) {
        const rightRay = sgWorld.Creator.CreatePolylineFromArray(verticesArray, pickRayInfo.isNothing ? 0xFF0000FF : 0xFFFFFFFF, 3, this.groupID, "ray");
        rightRay.SetParam(200, 0x200);  // Make sure that the ray object itself will not be pickable
        this.id = rightRay.ID;
      } else {
        try {
          const obj = Axiom.getObject(this.id, ObjectTypeCode.OT_POLYLINE);
          if (obj !== null) {
            obj.Geometry = sgWorld.Creator.GeometryCreator.CreateLineStringGeometry(verticesArray);
            obj.LineStyle.Color.abgrColor = (pickRayInfo.objectID !== undefined) ? 0xFF0000FF : 0xFFFF0000;
          }
        } catch (error) {
          Program.instance!.error("Ray error");
        }
      }
    }
  }

  export class Sphere {
    constructor(private groupID: string) { }
    id?: string;
    draw(pickRayInfo: collisionInfo) {
      const rayLengthScaleFactor = pickRayInfo.rayLength * 0.004;
      const sphereRadius = Math.max(0.01, rayLengthScaleFactor);
      const spherePivot = pickRayInfo.hitPoint.Copy();
      spherePivot.Altitude -= sphereRadius / 2;
      if (this.id == undefined) {
        const tip = sgWorld.Creator.CreateSphere(pickRayInfo.hitPoint.Copy(), sphereRadius, 0, 0x5000FF00, 0x5000FF00, 10, this.groupID, "rayTip");
        tip.SetParam(200, 0x200);
        this.id = tip.ID;
      } else {
        const obj = Axiom.getObject(this.id, ObjectTypeCode.OT_SPHERE);
        if (obj !== null) {
          obj.Position = pickRayInfo.hitPoint.Copy();
          obj.Position.Altitude -= sphereRadius / 2;
          obj.SetParam(200, 0x200); // not pickable
          obj.Radius = sphereRadius;
          obj.LineStyle.Color.FromARGBColor(pickRayInfo.objectID == undefined ? 0x50FFFFFF : 0x5000FF00);
        }
      }
    }
  }

  export class Laser {
    ray: Ray = new Ray(this.groupID);
    tip: Sphere = new Sphere(this.groupID);

    constructor(private groupID: string) { }

    collision?: collisionInfo;

    update() {
      const position = Program.instance!.controllerInfos[1].wandPosition;
      if (position === undefined)
        return;
      sgWorld.SetParam(8300, position); // Pick ray
      const hitObjectID = sgWorld.GetParam(8310) as string | undefined;
      let distToHitPoint = sgWorld.GetParam(8312) as number;    // Get distance to hit point
      let isNothing = false;
      if (distToHitPoint == 0) {
        distToHitPoint = sgWorld.Navigate.GetPosition(3).Altitude / 2;
        isNothing = true;
      }

      const hitPosition = position.Copy().Move(distToHitPoint, position.Yaw, position.Pitch);
      hitPosition.Cartesian = true;
      this.collision = {
        originPoint: position,
        hitPoint: hitPosition,
        rayLength: distToHitPoint,
        objectID: hitObjectID,
        isNothing: isNothing
      };
    }

    draw() {
      if (this.collision === undefined)
        return;
      this.ray.draw(this.collision);
      this.tip.draw(this.collision);
    }
  }

  export class Button {
    id: string;
    scale = 0.1;

    constructor(public name: string, public roomPositions: { [deviceType in DeviceType]: IPosition | null }, public modelPath: string, public callback: () => void, public groupID = "", public tooltip = "") {
      const obj = (() => {
        try {
          return sgWorld.Creator.CreateModel(SGWorld.Creator.CreatePosition(0, 0, 0, AltitudeTypeCode.ATC_TERRAIN_ABSOLUTE), this.modelPath, this.scale, 0, this.groupID, this.name);
        } catch {
          throw new Error(`Failed to create model with filename "${modelPath}" while attempting to construct button "${name}"`);
        }
      })();
      obj.BestLOD = 0;
      obj.Tooltip.Text = this.tooltip;
      obj.Visibility.Show = false;
      this.id = obj.ID;
    }

    update() {
      if (this.id === Program.instance!.getCollisionID() && Program.instance!.getButton1Pressed()) {
        this.callback();
        Program.instance!.setButton1Pressed(false);
      }
    }

    draw() {
      const roomPos = this.roomPositions[Program.instance!.getDeviceType()];
      if (roomPos === null)
        return;
      const pos = Axiom.roomToWorldCoord(roomPos);

      // Move the button to be in the right spot
      const obj = Axiom.getObject(this.id, ObjectTypeCode.OT_MODEL);
      if (obj === null) return;
      obj.Position = pos;
      obj.ScaleFactor = this.scale * (Program.instance!.controllerInfos[1].scaleFactor ?? 1.5);
      obj.Visibility.Show = true;
    }

    setScale(scale: number) {
      this.scale = scale;
    }

    show(value: boolean) {
      if (this.id === undefined) this.draw();
      if (this.id === undefined) return;
      const obj = Axiom.getObject(this.id, ObjectTypeCode.OT_MODEL);
      if (obj !== null)
        obj.Visibility.Show = value;
    }

    destroy() {
      Axiom.deleteItemSafe(this.id)
    }
  }

  export class Measurements {
    static init() {
      Measurements.group = getGroupID("measurements", getGroupID("Axiom"));
    }
    static first?: IPosition;
    static lineID?: string;
    static labelID?: string;
    static group?: string;
    static lineColor = sgWorld.Creator.CreateColor(255, 255, 0, 255);
    static labelStyle = sgWorld.Creator.CreateLabelStyle(0);
    static {
      Measurements.labelStyle.PivotAlignment = "Top";
      Measurements.labelStyle.MultilineJustification = "Left";
    }
    static measure() {
      if (Program.instance!.getButton2Pressed()) {
        // delete the measurement
        deleteItemSafe(Measurements.lineID);
        deleteItemSafe(Measurements.labelID);

        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[0].id);
        return;
      }
      if (Measurements.first === undefined || Measurements.lineID === undefined || Measurements.labelID === undefined) {
        Program.instance!.error("measurement info missing");
        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[0].id);
        return;
      }
      const end = Program.instance!.laser?.collision?.hitPoint.Copy();
      if (end === undefined) {
        Program.instance!.error("Lasert hitPoint missing")
        return;
      }
      // move line
      const start = Measurements.first.Copy().AimTo(end);
      const line = getObject(Measurements.lineID, ObjectTypeCode.OT_POLYLINE);
      if (line === null) {
        Program.instance!.errorText.writeLine("measurment line can't be found");
        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[0].id);
        return;
      }
      const geom = line.Geometry;
      geom.StartEdit();
      geom.Points.Item(1).X = end.X;
      geom.Points.Item(1).Y = end.Y;
      geom.EndEdit();

      // update label
      const direction: string = start.Yaw.toFixed(2);
      const distance: string = start.DistanceTo(end).toFixed(2);
      const labelText = `${direction} ${String.fromCharCode(176)} / ${distance}m`;
      const halfPos = start.Move(start.DistanceTo(end) / 2, start.Yaw, 0);
      const label = getObject(Measurements.labelID, ObjectTypeCode.OT_LABEL);
      if (label === null) return;
      label.Text = labelText;
      label.Position = halfPos;

      // Exit mode when pressed again
      if (Program.instance!.getButton1Pressed()) {
        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[0].id);
        Program.instance!.setButton1Pressed(false);
        Measurements.lineID = undefined;
        Measurements.labelID = undefined;
        Measurements.first = undefined;
      }
    }
    static measureStart() {
      if (Program.instance!.getButton2Pressed()) {
        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[0].id);
        return;
      }
      if (!Program.instance!.getButton1Pressed())
        return;
      // new line and label
      Measurements.first = Program.instance!.laser?.collision?.hitPoint.Copy();
      if (Measurements.first === undefined)
        return;

      const start = Measurements.first.Copy();
      const end = start.Copy();

      const lineWKT = `LineString( ${start.X} ${start.Y}, ${end.X} ${end.Y} )`;
      const lineGeom = sgWorld.Creator.GeometryCreator.CreateLineStringGeometry(lineWKT);
      const group = Measurements.group;
      if (group === undefined) {
        Program.instance!.error("no measurement group");
        return;
      }
      const line = sgWorld.Creator.CreatePolyline(lineGeom, Measurements.lineColor, AltitudeTypeCode.ATC_ON_TERRAIN, group, "line");
      line.LineStyle.Width = -10;
      const label = sgWorld.Creator.CreateTextLabel(start, "0m", Measurements.labelStyle, group, "label");

      Measurements.lineID = line.ID;
      Measurements.labelID = label.ID;

      Program.instance!.setButton1Pressed(false);

      Program.instance!.interactionMode = Mode.Measure;
    }
  }

  export class Drawings {
    static init() {
      Drawings.group = getGroupID("drawings", getGroupID("Axiom"));
    }
    static previous?: IPosition;
    static lineID?: string;
    static group?: string;

    static lineColor = sgWorld.Creator.CreateColor(0, 0, 0, 0); // black

    static line() {
      if (Drawings.previous === undefined || Drawings.lineID === undefined) {
        Program.instance!.errorText.writeLine("drawings info missing");
        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[1].id);
        return;
      }
      const end = Program.instance!.laser?.collision?.hitPoint.Copy();
      if (end === undefined) {
        Program.instance!.errorText.writeLine("Lasert hitPoint missing")
        return;
      }

      // move line
      const line = getObject(Drawings.lineID, ObjectTypeCode.OT_POLYLINE);
      if (line === null) {
        Program.instance!.errorText.writeLine("drawing line can't be found");
        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[1].id);
        return;
      }
      const geom = line.Geometry;
      geom.StartEdit();
      // Move the end point
      const drawPointIndex = geom.Points.Count - 1;
      geom.Points.Item(drawPointIndex).X = end.X;
      geom.Points.Item(drawPointIndex).Y = end.Y;

      if (Program.instance!.getButton1Pressed()) {
        // Always add a new segment when button1 pressed
        geom.Points.AddPoint(end.X, end.Y, end.Altitude);
        Drawings.previous = end;
      } else if (Program.instance!.getButton1()) {
        // dragging button1 will create segments when they are far enough apart
        if (end.DistanceTo(Drawings.previous) / (Program.instance!.controllerInfos[1].scaleFactor ?? 1) > 0.1) {
          geom.Points.AddPoint(end.X, end.Y, end.Altitude);
          Drawings.previous = end;
        }
      } else if (Program.instance!.getButton2Pressed()) {
        // button2 to stop
        geom.Points.DeletePoint(drawPointIndex);
        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[1].id);
      }

      geom.EndEdit();

      if (geom.Points.Count === 1)
        deleteItemSafe(Drawings.lineID);
    }
    static lineStart() {
      if (Program.instance!.getButton2Pressed()) {
        Program.instance!.interactionMode = Mode.None;
        Axiom.highlightById(false, Program.instance!.buttons[1].id);
        return;
      }
      if (!Program.instance!.getButton1Pressed())
        return;
      // new line
      Drawings.previous = Program.instance!.laser?.collision?.hitPoint.Copy();
      if (Drawings.previous === undefined)
        return;

      const start = Drawings.previous.Copy();
      const end = start.Copy();

      const lineWKT = "LineString( " + start.X + " " + start.Y + ", " + end.X + " " + end.Y + " )";
      const geom = sgWorld.Creator.GeometryCreator.CreateLineStringGeometry(lineWKT);
      const group = Drawings.group;
      if (group === undefined) {
        Program.instance!.error("no drawings group");
        return;
      }
      const line = sgWorld.Creator.CreatePolyline(geom, Drawings.lineColor, 2, group, "__line");
      line.LineStyle.Width = -10;
      Drawings.lineID = line.ID;

      Program.instance!.setButton1Pressed(false);

      Program.instance!.interactionMode = Mode.Line;
    }
  }
}

// Singleton
class Program {
  static instance?: Program;

  // create the Program instance
  static instantiate() {
    const folder = window.location.pathname.match(/^[\/\\]*(.*)[\/\\]/)?.[1];
    if (folder === undefined) {
      // Wait for folder
      const div = document.createElement("div");
      const label = document.createElement("label")
      const input = document.createElement("input");
      const button = document.createElement("button");

      label.textContent = "Please manually enter the folder";
      alert("Please manually enter the folder");
      input.focus();

      button.textContent = "Confirm";

      button.onclick = () => {
        div.parentNode?.removeChild(div);

        Program.instance = new Program(input.value);
      };

      div.appendChild(label);
      label.appendChild(input);
      div.appendChild(button);
      document.body.appendChild(div);
      return;
    }

    Program.instance = new Program(folder);
  }

  hasFolder = false;

  constructor(public folder: string) {
    this.runningText.writeLine(`folder = "${folder}"`);
    this.init();
  }

  errorText = new Axiom.TextElement();
  staticText = new Axiom.TextElement();
  runningText = new Axiom.TextElement();

  onSGWorld?: (eventID: number) => void;

  error(message: string) {
    this.errorText.writeLine(message);
    if (this.onSGWorld !== undefined) {
      sgWorld.DetachEvent("OnSGWorld", this.onSGWorld);
      this.onSGWorld = undefined;
    }
    throw new Error("Program Error Called");
  }

  startInfo?: {
    intersect: IPosition;
    prevWandRoomPos: MathUtils.Vector<3>;
    prevWandRoomDir: MathUtils.Vector<3>;
  };

  roomExtent?: {
    min: MathUtils.Vector<3>;
    max: MathUtils.Vector<3>;
  };

  deviceType = Axiom.DeviceType.desktop;

  controllerInfos: Partial<Axiom.ControllerInfo>[] = [{}, {}];

  laser?: Axiom.Laser;

  buttons: Axiom.Button[] = [];

  update() {
    const VRCstr = sgWorld.GetParam(8600) as string;
    const VRControllersInfo = (() => {
      try {
        return JSON.parse(VRCstr);
      } catch {
        this.error("ControllerInfo JSON parse error");
      }
    })();
    if (VRControllersInfo !== undefined) {
      for (let hand = 0; hand < 2; ++hand) {
        const prevTrigger = this.controllerInfos[hand]?.trigger ?? false;
        const prevButton1 = this.controllerInfos[hand]?.button1 ?? false;
        const prevButton2 = this.controllerInfos[hand]?.button2 ?? false;

        this.controllerInfos[hand] = {};

        const triggerOn = VRControllersInfo.IndexTrigger && VRControllersInfo.IndexTrigger[hand] != 0
        const button1On = (VRControllersInfo.Buttons & [0x200, 0x2][hand]) != 0;
        const button2On = (VRControllersInfo.Buttons & [0x100, 0x1][hand]) != 0;

        this.controllerInfos[hand].triggerPressed = triggerOn && !prevTrigger;
        this.controllerInfos[hand].button1Pressed = button1On && !prevButton1;
        this.controllerInfos[hand].button2Pressed = button2On && !prevButton2;

        this.controllerInfos[hand].trigger = triggerOn;
        this.controllerInfos[hand].button1 = button1On;
        this.controllerInfos[hand].button2 = button2On;

        if (VRControllersInfo.HavePosition != undefined && VRControllersInfo.HavePosition[hand]) {
          const wandPosition = sgWorld.Navigate.GetPosition(3);
          wandPosition.Distance = 100000;
          wandPosition.X = VRControllersInfo.Position[hand][0];
          wandPosition.Y = VRControllersInfo.Position[hand][2];
          wandPosition.Altitude = VRControllersInfo.Position[hand][1];
          if (VRControllersInfo.HaveOrientation != undefined && VRControllersInfo.HaveOrientation[hand]) {
            wandPosition.Yaw = VRControllersInfo.Yaw[hand];
            wandPosition.Pitch = VRControllersInfo.Pitch[hand];
            wandPosition.Roll = VRControllersInfo.Roll[hand];
            const headPosition = sgWorld.Navigate.GetPosition(3);
            const tmpHeadsetpos = sgWorld.GetParam(8601) as IPosition;
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
    } else {
      this.controllerInfos[0].triggerPressed = false;
      this.controllerInfos[0].button1Pressed = false;
      this.controllerInfos[0].button2Pressed = false;
      this.controllerInfos[1].triggerPressed = false;
      this.controllerInfos[1].button1Pressed = false;
      this.controllerInfos[1].button2Pressed = false;
    }
    if (this.roomExtent === undefined) {
      const extent = sgWorld.SetParamEx(9015) as string; // get the VR controls status
      const roomExtent = JSON.parse(extent);
      this.roomExtent = {
        min: new MathUtils.Vector<3>([roomExtent.minX, roomExtent.minY, roomExtent.minZ]),
        max: new MathUtils.Vector<3>([roomExtent.maxX, roomExtent.maxY, roomExtent.maxZ])
      };
    }
    this.laser?.update();
    for (let button of this.buttons) {
      button.draw();
      button.update();
    }
    this.laser?.draw();
    this.dragMode();
    this.interact();
  }

  tableFrame(eventID: number) {
    try {
      if (eventID == 14)
        this.update();
    } catch {
      this.error("Error on table frame");
    }
  }

  firstTableFrame(eventID: number) {
    try {
      this.runningText.writeLine("first frame");
      if (eventID == 14) {
        Axiom.setComClientForcedInputMode();
        sgWorld.DetachEvent("OnSGWorld", this.onSGWorld!);
        this.onSGWorld = this.tableFrame.bind(this);
        sgWorld.AttachEvent("OnSGWorld", this.onSGWorld);
      }
    } catch {
      this.error("Error on first table frame");
    }
  }

  init() {
    this.runningText.writeLine("init");
    Axiom.deleteGroup("Axiom");
    Axiom.Measurements.init();
    Axiom.Drawings.init();
    this.createLaser();
    this.createButtons();

    this.onSGWorld = this.firstTableFrame.bind(this);
    sgWorld.AttachEvent("OnSGWorld", this.onSGWorld);
  }

  private createLaser() {
    this.laser = new Axiom.Laser(Axiom.getGroupID("Laser", Axiom.getGroupID("Axiom")));
  }

  private createButtons() {
    const group = Axiom.getGroupID("Buttons", Axiom.getGroupID("Axiom"));
    this.buttons.push(new Axiom.Button("Measure", {
      [Axiom.DeviceType.desktop]: null,
      [Axiom.DeviceType.table]: sgWorld.Creator.CreatePosition(-0.5, -1.1, 0.65, AltitudeTypeCode.ATC_TERRAIN_ABSOLUTE),
      [Axiom.DeviceType.wall]: sgWorld.Creator.CreatePosition(-0.9, 0, 1.1, AltitudeTypeCode.ATC_TERRAIN_ABSOLUTE)
    }, this.folder + "/measure.xpl2", this.pushMeasure.bind(this), group));
    this.buttons.push(new Axiom.Button("Line", {
      [Axiom.DeviceType.desktop]: null,
      [Axiom.DeviceType.table]: sgWorld.Creator.CreatePosition(-0.4, -1.1, 0.65, AltitudeTypeCode.ATC_TERRAIN_ABSOLUTE),
      [Axiom.DeviceType.wall]: sgWorld.Creator.CreatePosition(-0.9, 0, 1, AltitudeTypeCode.ATC_TERRAIN_ABSOLUTE)
    }, this.folder + "/line.xpl2", this.pushLine.bind(this), group));
  }

  interactionMode = Axiom.Mode.None;

  pushMeasure() {
    if (this.interactionMode !== Axiom.Mode.None)
      return;
    Axiom.highlightById(true, this.buttons[0].id);
    this.runningText.writeLine("Measure");
    this.interactionMode = Axiom.Mode.MeasureStart;
  }

  pushLine() {
    if (this.interactionMode !== Axiom.Mode.None)
      return;
    Axiom.highlightById(true, this.buttons[1].id);
    this.runningText.writeLine("Line");
    this.interactionMode = Axiom.Mode.LineStart;
  }

  interact() {
    switch (this.interactionMode) {
      case Axiom.Mode.MeasureStart: Axiom.Measurements.measureStart(); break;
      case Axiom.Mode.Measure: Axiom.Measurements.measure(); break;
      case Axiom.Mode.LineStart: Axiom.Drawings.lineStart(); break;
      case Axiom.Mode.Line: Axiom.Drawings.line(); break;
    }
  }

  getButton3() { return this.controllerInfos[1]?.trigger ?? false; }
  getCollisionPosition() { return this.laser?.collision?.hitPoint; }
  getWandPosition() { return this.laser?.collision?.originPoint; }
  getCollisionID() { return this.laser?.collision?.objectID; }
  getButton1Pressed() { return this.controllerInfos[1]?.button1Pressed ?? false; }
  getButton1() { return this.controllerInfos[1]?.button1 ?? false; }
  getButton2Pressed() { return this.controllerInfos[1]?.button2Pressed ?? false; }
  setButton1Pressed(pressed: boolean) { (this.controllerInfos[1] ??= {}).button1Pressed = pressed; }

  getDeviceType() {
    if (this.deviceType === Axiom.DeviceType.desktop && this.roomExtent?.max !== undefined) {
      if (this.roomExtent?.max.data[2] > 1.9)
        this.deviceType = Axiom.DeviceType.wall;
      else
        this.deviceType = Axiom.DeviceType.table;
    }
    return this.deviceType;
  }

  dragMode() {
    const trigger = this.getButton3();
    let newIntersect = this.getCollisionPosition();
    const wandWorldIPos = this.getWandPosition();

    if (wandWorldIPos === undefined)
      throw new Error("Unable to find wand position");
    const wandRoomIPos = Axiom.worldToRoomCoord(wandWorldIPos);

    const wandOri1 = MathUtils.Quaternion.fromYPR(-MathUtils.degsToRads(wandRoomIPos.Yaw), MathUtils.degsToRads(wandRoomIPos.Pitch), MathUtils.degsToRads(wandRoomIPos.Roll));
    // orientation is wrong on the wall. Pitch down
    const wandOri = wandOri1;
    const wandRoomDir = wandOri.getYAxis(1);
    const wandRoomPos = new MathUtils.Vector<3>([wandRoomIPos.X, wandRoomIPos.Y, wandRoomIPos.Altitude]);
    this.staticText.setText(wandRoomPos.data.map(x => x.toFixed(2)).join(", "));

    if (newIntersect === undefined) {
      return;
    }
    const roomIntersect = Axiom.worldToRoomCoord(newIntersect);
    const roomX = roomIntersect.X;
    const roomY = this.getDeviceType() === Axiom.DeviceType.table ? roomIntersect.Y : roomIntersect.Altitude;
    if (this.roomExtent !== undefined) {
      const minX = this.roomExtent.min.data[0];
      const maxX = this.roomExtent.max.data[0];
      const minY = this.roomExtent.min.data[this.getDeviceType() == Axiom.DeviceType.table ? 1 : 2];
      const maxY = this.roomExtent.max.data[this.getDeviceType() == Axiom.DeviceType.table ? 1 : 2];
      const deadzone = 1;
      if (roomX < minX - deadzone || roomX > maxX + deadzone || roomY < minY - deadzone || roomY > maxY + deadzone)
        return;
    }
    if (this.startInfo !== undefined) {
      if (!trigger) {
        this.startInfo = undefined;
        return;
      }
      // dragged!
      const worldPos = sgWorld.Navigate.GetPosition(3).Copy();

      // zoom
      const wandPosDiff = wandRoomPos.copy().sub(this.startInfo.prevWandRoomPos);
      const magDifference = wandPosDiff.mag();
      if (magDifference > 0 && magDifference < 1) {
        let forwardOrBack = wandPosDiff.dot(this.startInfo.prevWandRoomDir);
        forwardOrBack = forwardOrBack >= 0 ? 1 : -1;
        let scaleRatio = 5;

        const degs = MathUtils.radsToDegs(Math.acos(Math.abs(wandPosDiff.copy().normalise().dot(this.startInfo.prevWandRoomDir.copy().normalise()))));
        const thresholdLower = 25;
        const thresholdUpper = 40;
        const thresholdRange = thresholdUpper - thresholdLower;
        const scalingRatio = 1 - Math.min(Math.max(degs, thresholdLower) - thresholdLower, thresholdRange) / thresholdRange;

        let power = forwardOrBack * scalingRatio * magDifference * 2;
        let powerStrength = 1;
        if (this.getDeviceType() == Axiom.DeviceType.wall) {
          let curAltitudeRatio = worldPos.Altitude / 2000; // scaling now works less as you go from 1000 down to 10 altitude
          powerStrength = Math.min(Math.max(curAltitudeRatio, 0.25), 1);
        }
        const factor = Math.pow(scaleRatio, power * powerStrength);
        worldPos.Altitude *= factor;
        // newIntersect should move away from the center of the screen by zoom factor
        newIntersect.Cartesian = true;
        newIntersect = newIntersect.MoveToward(sgWorld.Navigate.GetPosition(3), (1 - factor) * newIntersect.DistanceTo(sgWorld.Navigate.GetPosition(3)));

        const maxTableAltitude = 500000;
        const maxWallAltitude = 60000;
        if (this.getDeviceType() === Axiom.DeviceType.table && worldPos.Altitude > maxTableAltitude) {
          worldPos.Altitude = maxTableAltitude;
        } else if (this.getDeviceType() === Axiom.DeviceType.wall && worldPos.Altitude > maxWallAltitude) {
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
    } else if (trigger) {
      this.startInfo = {
        intersect: newIntersect,
        prevWandRoomPos: wandRoomPos,
        prevWandRoomDir: wandRoomDir
      }
    }
  }
}
