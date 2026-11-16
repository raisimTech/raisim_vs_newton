"""Create exportable FPS and FPS/USD charts from recorded runs (optional matplotlib)."""
import argparse
import json
from pathlib import Path


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=Path('plots'))
    args=parser.parse_args()
    rows=json.loads(args.results.read_text())
    args.output.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'svg.fonttype':'none','text.parse_math':False})
    for model in sorted({r['model'] for r in rows}):
        subset=[r for r in rows if r['model']==model];worlds=sorted({r['worlds'] for r in subset})
        fig,axes=plt.subplots(1,2,figsize=(14,5))
        for ax,key,label in zip(axes,['aggregate_fps','fps_per_usd'],['Aggregate physics steps/s','Physics steps/s per USD']):
            for backend,offset,color in [('raisim',-.19,'#00846e'),('newton',.19,'#8c49ba')]:
                values=[next((r[key] for r in subset if r['backend']==backend and r['worlds']==n),np.nan) for n in worlds]
                ax.bar(np.arange(len(worlds))+offset,values,.35,label=backend,color=color)
            ax.set_xticks(range(len(worlds)),[f'{n:,}' for n in worlds],rotation=35,ha='right')
            ax.set_xlabel('Independent worlds');ax.set_ylabel(label);ax.legend();ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        fig.suptitle(f'{model}: random targets, ten physics steps per control interval')
        fig.text(.02,.01,f"CPU USD {subset[0]['cpu_price_usd']:,.2f}; GPU USD {subset[0]['gpu_price_usd']:,.2f}. Component-only prices; no policy inference/rewards/resets.",fontsize=9)
        fig.tight_layout(rect=[0,.05,1,.95]);fig.savefig(args.output/f'{model}.svg',metadata={'Date':None});fig.savefig(args.output/f'{model}.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
