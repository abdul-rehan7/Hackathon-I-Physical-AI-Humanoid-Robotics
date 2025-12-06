import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className={clsx(styles.buttonPrimary)}
              to="/docs/intro">
              Start Reading
            </Link>
            <Link
              className={clsx(styles.buttonSecondary)}
              to="/docs/hardware-requirements">
              Learn More
            </Link>
          </div>
        </div>
        <div className={styles.heroIllustration}>
          <div className={styles.gradientBox}></div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ icon, title, description }: { icon: string; title: string; description: string }) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function Features() {
  return (
    <section className={styles.featuresSection}>
      <div className={styles.featuresContainer}>
        <h2 className={styles.sectionTitle}>Key Topics</h2>
        <div className={styles.featuresGrid}>
          <FeatureCard
            icon="🤖"
            title="Robotics Fundamentals"
            description="Learn the core principles of robotics, kinematics, and dynamics essential for humanoid robot development."
          />
          <FeatureCard
            icon="🧠"
            title="AI Integration"
            description="Explore artificial intelligence techniques for perception, planning, and decision-making in robotic systems."
          />
          <FeatureCard
            icon="⚡"
            title="Hardware Architecture"
            description="Understand the hardware requirements and design considerations for building physical humanoid robots."
          />
          <FeatureCard
            icon="💡"
            title="Practical Implementation"
            description="Gain hands-on experience with real-world robotics projects and practical implementation strategies."
          />
          <FeatureCard
            icon="📚"
            title="Comprehensive Guide"
            description="A structured learning path through 13 weeks of in-depth content covering all aspects of Physical AI."
          />
          <FeatureCard
            icon="🚀"
            title="Future-Ready Skills"
            description="Develop cutting-edge skills in one of the most innovative and rapidly growing fields of technology."
          />
        </div>
      </div>
    </section>
  );
}

function CTA() {
  return (
    <section className={styles.ctaSection}>
      <div className={styles.ctaContent}>
        <h2 className={styles.ctaTitle}>Ready to Learn?</h2>
        <p className={styles.ctaDescription}>
          Dive into our comprehensive guide to Physical AI and Humanoid Robotics. Start your journey today!
        </p>
        <Link
          className={clsx(styles.ctaButton)}
          to="/docs/intro">
          Begin Learning
        </Link>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Learn Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <Features />
        <CTA />
      </main>
    </Layout>
  );
}
