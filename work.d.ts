declare const SGWorld: ISGWorld;
declare const sgWorld: ISGWorld;
declare namespace MathUtils {
    type Shift<A extends Array<any>> = ((...args: A) => void) extends ((...args: [A[0], ...infer R]) => void) ? R : never;
    type GrowExpRev<A extends Array<any>, N extends number, P extends Array<Array<any>>> = A['length'] extends N ? A : {
        0: GrowExpRev<[...A, ...P[0]], N, P>;
        1: GrowExpRev<A, N, Shift<P>>;
    }[[...A, ...P[0]][N] extends undefined ? 0 : 1];
    type GrowExp<A extends Array<any>, N extends number, P extends Array<Array<any>>> = A['length'] extends N ? A : {
        0: GrowExp<[...A, ...A], N, [A, ...P]>;
        1: GrowExpRev<A, N, P>;
    }[[...A, ...A][N] extends undefined ? 0 : 1];
    export type FixedSizeArray<T, N extends number> = N extends 0 ? [] : N extends 1 ? [T] : GrowExp<[T, T], N, [[T]]>;
    export class Vector<N extends number> {
        data: FixedSizeArray<number, N>;
        constructor(data: FixedSizeArray<number, N>);
        copy(): Vector<N>;
        add(b: Vector<N>): Vector<N>;
        sub(b: Vector<N>): Vector<N>;
        mul(v: number): Vector<N>;
        dot(b: Vector<N>): number;
        cross(b: Vector<3>): Vector<3>;
        equals(b: Vector<N>): boolean;
        mag(): number;
        normalise(): Vector<N>;
    }
    export class Quaternion {
        data: FixedSizeArray<number, 4>;
        constructor(data: FixedSizeArray<number, 4>);
        static fromYPR(yRad: number, pRad: number, rRad: number): Quaternion;
        copy(): Quaternion;
        zeroRoll(): Quaternion;
        getYPR(): number[];
        getRoll(): number;
        getPitch(): number;
        getYaw(): number;
        conjugate(): Quaternion;
        getXAxis(v: number): Vector<3>;
        getYAxis(v: number): Vector<3>;
        getZAxis(v: number): Vector<3>;
        postApplyXAxis(v: number): Quaternion;
        postApplyYAxis(v: number): Quaternion;
        postApplyZAxis(v: number): Quaternion;
        preApplyXAxis(v: number): Quaternion;
        preApplyYAxis(v: number): Quaternion;
        preApplyZAxis(v: number): Quaternion;
        apply(v: Vector<3>): Vector<3>;
        mul(q: Quaternion): this;
        equals(b: Quaternion): boolean;
        mag(): number;
        normalise(): Quaternion;
    }
    export function degsToRads(degs: number): number;
    export function radsToDegs(degs: number): number;
    export {};
}
declare namespace Axiom {
    class TextElement {
        element: HTMLElement;
        constructor();
        writeLine(message: string): void;
        setText(message: string): void;
    }
    function worldToRoomCoord(position: IPosition): IPosition;
    function roomToWorldCoord(position: IPosition): IPosition;
    function setComClientForcedInputMode(): void;
    interface ControllerInfo {
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
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_IMAGERY_LAYER): ITerrainRasterLayer | null;
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_MODEL): ITerrainModel | null;
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_POLYLINE): ITerrainPolyline | null;
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_LABEL): ITerrainLabel | null;
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_POLYGON): ITerrainPolygon | null;
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_SPHERE): ITerrainSphere | null;
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_BOX): ITerrain3DRectBase | null;
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_IMAGE_LABEL): ITerrainImageLabel | null;
    function getObject(oid: string | undefined, objectType: ObjectTypeCode.OT_RECTANGLE): ITerrainRectangle | null;
    function getGroupID(groupName: string, parentGroup?: string): string;
    function deleteGroup(groupName: string): boolean;
    function deleteItemSafe(id: string | undefined): void;
    function highlightById(highlight: boolean, oid: string): void;
    const enum DeviceType {
        table = 0,
        wall = 1,
        desktop = 2
    }
    const enum Mode {
        None = 0,
        MeasureStart = 1,
        Measure = 2,
        LineStart = 3,
        Line = 4
    }
    interface ControllerInfo {
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
    type collisionInfo = {
        originPoint: IPosition;
        hitPoint: IPosition;
        rayLength: number;
        objectID?: string;
        isNothing: boolean;
    };
    class Ray {
        private groupID;
        constructor(groupID: string);
        id?: string;
        draw(pickRayInfo: collisionInfo): void;
    }
    class Sphere {
        private groupID;
        constructor(groupID: string);
        id?: string;
        draw(pickRayInfo: collisionInfo): void;
    }
    class Laser {
        private groupID;
        ray: Ray;
        tip: Sphere;
        constructor(groupID: string);
        collision?: collisionInfo;
        update(): void;
        draw(): void;
    }
    class Button {
        name: string;
        roomPositions: {
            [deviceType in DeviceType]: IPosition | null;
        };
        modelPath: string;
        callback: () => void;
        groupID: string;
        tooltip: string;
        id: string;
        scale: number;
        constructor(name: string, roomPositions: {
            [deviceType in DeviceType]: IPosition | null;
        }, modelPath: string, callback: () => void, groupID?: string, tooltip?: string);
        update(): void;
        draw(): void;
        setScale(scale: number): void;
        show(value: boolean): void;
        destroy(): void;
    }
    class Measurements {
        static init(): void;
        static first?: IPosition;
        static lineID?: string;
        static labelID?: string;
        static group?: string;
        static lineColor: IColor;
        static labelStyle: ILabelStyle;
        static measure(): void;
        static measureStart(): void;
    }
    class Drawings {
        static init(): void;
        static previous?: IPosition;
        static lineID?: string;
        static group?: string;
        static lineColor: IColor;
        static line(): void;
        static lineStart(): void;
    }
}
declare class Program {
    folder: string;
    static instance?: Program;
    static instantiate(): void;
    hasFolder: boolean;
    constructor(folder: string);
    errorText: Axiom.TextElement;
    staticText: Axiom.TextElement;
    runningText: Axiom.TextElement;
    onSGWorld?: (eventID: number) => void;
    error(message: string): void;
    startInfo?: {
        intersect: IPosition;
        prevWandRoomPos: MathUtils.Vector<3>;
        prevWandRoomDir: MathUtils.Vector<3>;
    };
    roomExtent?: {
        min: MathUtils.Vector<3>;
        max: MathUtils.Vector<3>;
    };
    deviceType: Axiom.DeviceType;
    controllerInfos: Partial<Axiom.ControllerInfo>[];
    laser?: Axiom.Laser;
    buttons: Axiom.Button[];
    update(): void;
    tableFrame(eventID: number): void;
    firstTableFrame(eventID: number): void;
    init(): void;
    private createLaser;
    private createButtons;
    interactionMode: Axiom.Mode;
    pushMeasure(): void;
    pushLine(): void;
    interact(): void;
    getButton3(): boolean;
    getCollisionPosition(): IPosition | undefined;
    getWandPosition(): IPosition | undefined;
    getCollisionID(): string | undefined;
    getButton1Pressed(): boolean;
    getButton1(): boolean;
    getButton2Pressed(): boolean;
    setButton1Pressed(pressed: boolean): void;
    getDeviceType(): Axiom.DeviceType;
    dragMode(): void;
}
