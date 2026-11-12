#!/usr/bin/env python3
"""Export measured run statistics and all-frame pose comparisons to JSON and Excel."""
import argparse
import csv
import json
import math
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
from xml.sax.saxutils import escape

import numpy as np
from align_trajectory_sim3 import read_csv
from render_trajectory_video import euler_deg


def write_xlsx(path, sheets):
    # OOXML inline strings preserve nanosecond timestamps beyond Excel's 15-digit precision.
    def column(n):
        value = ''
        while n:
            n, r = divmod(n - 1, 26)
            value = chr(65 + r) + value
        return value
    ns = 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'
    with ZipFile(path, 'w', ZIP_DEFLATED) as archive:
        overrides = ''.join(f'<Override PartName="/xl/worksheets/sheet{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>' for i in range(1, len(sheets)+1))
        archive.writestr('[Content_Types].xml', '<?xml version="1.0"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'+overrides+'</Types>')
        archive.writestr('_rels/.rels', '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>')
        archive.writestr('xl/workbook.xml', f'<workbook xmlns="{ns}" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets>'+''.join(f'<sheet name="{name}" sheetId="{i}" r:id="rId{i}"/>' for i,(name,_) in enumerate(sheets,1))+'</sheets></workbook>')
        archive.writestr('xl/_rels/workbook.xml.rels', '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'+''.join(f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet{i}.xml"/>' for i in range(1,len(sheets)+1))+'</Relationships>')
        for number, (_, rows) in enumerate(sheets,1):
            with archive.open(f'xl/worksheets/sheet{number}.xml','w') as stream:
                stream.write(f'<worksheet xmlns="{ns}"><sheetViews><sheetView workbookViewId="0"><pane ySplit="1" topLeftCell="A2" activePane="bottomLeft" state="frozen"/></sheetView></sheetViews><sheetData>'.encode())
                width = 1
                for index, row in enumerate(rows,1):
                    width = max(width,len(row)); cells=[]
                    for j,value in enumerate(row,1):
                        ref=f'{column(j)}{index}'
                        if value is None: continue
                        if isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value):
                            cells.append(f'<c r="{ref}"><v>{value}</v></c>')
                        else:
                            cells.append(f'<c r="{ref}" t="inlineStr"><is><t>{escape(str(value))}</t></is></c>')
                    stream.write((f'<row r="{index}">'+''.join(cells)+'</row>').encode())
                stream.write(f'</sheetData><autoFilter ref="A1:{column(width)}{index}"/></worksheet>'.encode())


def stats(values):
    a=np.array([float(x) for x in values if x is not None and str(x)!=''],dtype=float)
    a=a[np.isfinite(a)]
    if not len(a): return dict(count=0)
    return dict(count=len(a),mean=float(a.mean()),median=float(np.median(a)),
                std=float(a.std()),min=float(a.min()),max=float(a.max()),
                p90=float(np.percentile(a,90)),p95=float(np.percentile(a,95)),
                p99=float(np.percentile(a,99)),rmse=float(np.sqrt(np.mean(a*a))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--comparison-dir',type=Path)
    args=parser.parse_args(); run=args.run_dir
    comp=args.comparison_dir or run/'comparison'
    metrics=json.loads((run/'metrics.json').read_text())
    alignment=json.loads((comp/'alignment.json').read_text())
    raw={int(r['stamp_ns']):r for r in read_csv(run/'trajectory.csv')}
    aligned={int(r['frame_index']):r for r in read_csv(comp/'trajectory_aligned.csv')}
    gt={r['Label']:r for r in read_csv(alignment['ground_truth'])}
    truth={int(r['frame_index']):r for r in read_csv(comp/'ground_truth.csv')}
    diagnostics_path=run/'trajectory.frames.csv'
    diagnostics={int(r['stamp_ns']):r for r in read_csv(diagnostics_path)} if diagnostics_path.exists() else {}
    interval=round(1e9/alignment.get('source_fps',30.0))
    count=alignment['source_frames']; rows=[]; previous=None
    for i in range(count):
        stamp=alignment['first_image_stamp_ns']+i*interval
        a=aligned.get(i); t=truth.get(i); diag=diagnostics.get(stamp,{})
        label=a['label'] if a else (t['label'] if t else None)
        row=dict(frame_index=i,image=label,stamp_ns=str(stamp),
                 timestamp_basis='synthetic_source_fps',synchronization=alignment['synchronization'],
                 relative_source_time_s=i*interval/1e9,
                 status=diag.get('status', 'TRACKING' if a else 'NO_POSE_STATUS_UNAVAILABLE'),
                 status_basis='recorded' if diag else 'inferred_from_odometry_presence',
                 has_pose=bool(a),has_ground_truth=bool(t),
                 processing_ms=float(diag['processing_ms']) if diag.get('processing_ms') else None,
                 track_ms=float(diag['track_ms']) if diag.get('track_ms') else None,
                 completed_wall_ns=diag.get('completed_wall_ns'),position_error=None,
                 horizontal_error=None,vertical_abs_error=None,rpe_translation_step=None,
                 orientation_error_deg=None)
        if a:
            original=raw[int(a['stamp_ns'])]
            for k in ('x','y','z','qx','qy','qz','qw'):
                row['raw_'+k]=float(original[k]);row['aligned_'+k]=float(a[k])
            angles=euler_deg(*[float(a[k]) for k in ('qx','qy','qz','qw')])
            row.update(zip(('aligned_roll_deg','aligned_pitch_deg','aligned_yaw_deg'),angles))
        if t:
            for k in ('x','y','z'):row['gt_'+k]=float(t[k])
            for k in ('Omega','Phi','Kappa'):
                value=gt.get(label,{}).get(k)
                row['gt_'+k+'_deg']=float(value) if value else None
        if a and t:
            error=np.array([float(a[k])-float(t[k]) for k in ('x','y','z')])
            row.update(zip(('error_x','error_y','error_z'),error.tolist()))
            row.update(position_error=float(np.linalg.norm(error)),horizontal_error=float(np.linalg.norm(error[:2])),vertical_abs_error=abs(float(error[2])))
            if previous is not None and previous[0]==i-1:
                row['rpe_translation_step']=float(np.linalg.norm(error-previous[1]))
            previous=(i,error)
        else:previous=None
        rows.append(row)
    columns=list(dict.fromkeys(k for row in rows for k in row))
    with (comp/'per_frame_results.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=columns);w.writeheader();w.writerows(rows)
    runs=[];length=0
    for row in rows:
        if row['has_pose']:length+=1
        elif length:runs.append(length);length=0
    if length:runs.append(length)
    def path_length(prefix):
        total=0.0
        for a,b in zip(rows,rows[1:]):
            if all(a.get(prefix+k) is not None and b.get(prefix+k) is not None for k in ('x','y','z')):
                total+=math.sqrt(sum((b[prefix+k]-a[prefix+k])**2 for k in ('x','y','z')))
        return total
    states={}
    for row in rows:states[row['status']]=states.get(row['status'],0)+1
    report=dict(run_metrics=metrics,alignment=alignment,source_frames=count,
                valid_pose_frames=len(aligned),missing_pose_frames=count-len(aligned),
                pose_coverage=len(aligned)/count,tracking_segments=len(runs),
                longest_pose_segment=max(runs,default=0),diagnostic_frames=len(diagnostics),
                per_frame_status_counts=states,
                gt_path_length=path_length('gt_'),aligned_path_length_on_valid_adjacent_frames=path_length('aligned_'),
                source_duration_s=(count-1)*interval/1e9,
                measured_frame_metrics={k:stats(r.get(k) for r in rows) for k in
                 ('processing_ms','track_ms','position_error','horizontal_error','vertical_abs_error',
                  'error_x','error_y','error_z','rpe_translation_step')},
                notes=['Position errors use GT coordinate units and one globally fitted Sim3.',
                       'Errors are measured on the same poses used to fit Sim3; missing poses are excluded from accuracy, but counted in coverage.',
                       'RPE here is one-source-frame relative translation vector error; no gaps are bridged.',
                       'processing_ms covers the tracker callback, not PNG decoding or end-to-end latency; track_ms is cuVSLAM API wall time, not GPU kernel timing.',
                       'Source timestamps are synthesized from source_fps, not original camera hardware timestamps.',
                       'EO Omega/Phi/Kappa rotation and camera-axis conventions are unconfirmed. They are preserved separately; orientation error is blank.',
                       'No unavailable AUC, reprojection errors, landmark counts, GPU utilization, or orientation accuracy are invented.'])
    (comp/'results_report.json').write_text(json.dumps(report,indent=2)+'\n')
    summary=[['metric','value']]
    def flatten(value,prefix=''):
        if isinstance(value,dict):
            for k,v in value.items():flatten(v,prefix+'.'+k if prefix else k)
        else:summary.append([prefix,json.dumps(value) if isinstance(value,list) else value])
    flatten(report)
    definitions=[['field','meaning'],['stamp_ns','Text to preserve nanosecond precision; synthetic source timestamp'],
                 ['aligned_*','Position/orientation in GT frame after global Sim3'],
                 ['gt_Omega/Phi/Kappa_deg','Original EO angles; not interchangeable with roll/pitch/yaw'],
                 ['orientation_error_deg','Unavailable until EO angle/axis convention is confirmed'],
                 ['NO_POSE_STATUS_UNAVAILABLE','Old data has no odometry; original status was not recorded'],
                 ['rpe_translation_step','Norm of difference between adjacent aligned and GT displacement vectors'],
                 ['accuracy_units','Same units as truth coordinates; blank cells mean unavailable']]
    write_xlsx(comp/'per_frame_results.xlsx',[('Summary',summary),('Frames',[columns]+[[r.get(k) for k in columns] for r in rows]),('Definitions',definitions)])
    (comp/'results_report.txt').write_text('\n'.join(f'{k}: {v}' for k,v in summary[1:])+'\n')
    print(f'Report and Excel: {comp}; frames={len(rows)}, diagnostics={len(diagnostics)}')


if __name__=='__main__':main()
