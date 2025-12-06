import React, {useEffect, useRef} from 'react';
import type {Props} from '@theme/Root';

function ScrollProgressBar(): React.ReactElement | null {
  const barRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    if (typeof window === 'undefined') return undefined;

    const update = () => {
      const scrollTop = window.scrollY || document.documentElement.scrollTop;
      const docHeight = document.documentElement.scrollHeight - document.documentElement.clientHeight;
      const progress = docHeight > 0 ? (scrollTop / docHeight) * 100 : 0;
      if (barRef.current) {
        barRef.current.style.width = `${Math.min(Math.max(progress, 0), 100)}%`;
      }
    };

    const onScroll = () => {
      window.requestAnimationFrame(update);
    };

    update();
    window.addEventListener('scroll', onScroll, {passive: true});
    window.addEventListener('resize', update);
    return () => {
      window.removeEventListener('scroll', onScroll);
      window.removeEventListener('resize', update);
    };
  }, []);

  return <div className="scroll-progress-bar" ref={barRef} />;
}

export default function Root({children}: Props): React.ReactElement {
  return (
    <>
      <ScrollProgressBar />
      {children}
    </>
  );
}
